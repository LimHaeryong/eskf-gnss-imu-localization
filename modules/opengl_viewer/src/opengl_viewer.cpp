#include <chrono>
#include <thread>

#include "opengl_viewer/opengl_viewer.h"

OpenglViewer::OpenglViewer()
    : mPointQueue(std::make_shared<ThreadsafeQueue<std::shared_ptr<Point>>>(500))
    {}

int OpenglViewer::run()
{
    SPDLOG_INFO("Start program");

    SPDLOG_INFO("Initialize glfw");
    if (!glfwInit())
    {
        const char *description = nullptr;
        glfwGetError(&description);
        SPDLOG_ERROR("failed to initialize glfw: {}", description);
        return -1;
    }
    // glfwInit() 호출 직후
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    SPDLOG_INFO("Create glfw window");
    auto window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "OpenGL_Window", nullptr, nullptr);
    if (!window)
    {
        SPDLOG_ERROR("failed to create glfw window");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window); // window를 만든 직후
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        SPDLOG_ERROR("failed to initialize glad");
        glfwTerminate();
        return -1;
    }
    auto glVersion = glGetString(GL_VERSION);
    SPDLOG_INFO("OpenGL context version: {}", (const char *)glVersion);

    auto imguiContext = ImGui::CreateContext();
    ImGui::SetCurrentContext(imguiContext);
    ImGui_ImplGlfw_InitForOpenGL(window, false);    
    ImGui_ImplOpenGL3_Init();
    ImGui_ImplOpenGL3_CreateFontsTexture();
    ImGui_ImplOpenGL3_CreateDeviceObjects();    

    auto context = Context::create();
    if(!context)
    {
        SPDLOG_ERROR("failed to create context");
        glfwTerminate();
        return -1;
    }
    glfwSetWindowUserPointer(window, context.get());

    onFrameBufferSizeChange(window, WINDOW_WIDTH, WINDOW_HEIGHT);
    glfwSetFramebufferSizeCallback(window, onFrameBufferSizeChange);
    glfwSetKeyCallback(window, onKeyEvent);
    glfwSetCursorPosCallback(window, onCursorPos);
    glfwSetMouseButtonCallback(window, onMouseButton);
    glfwSetCharCallback(window, onCharacterEvent);
    glfwSetScrollCallback(window, onScroll);

    auto loopDuration = std::chrono::duration<double>(0.03);
    auto prevTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedTime, sleepTime;

    SPDLOG_INFO("Start main loop");
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        while(!mPointQueue->empty())
        {
            auto Point = mPointQueue->pop();
            context->addPoint(*Point);
        }

        context->processInput(window);
        context->render();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);

        elapsedTime = std::chrono::high_resolution_clock::now() - prevTime;
        sleepTime = loopDuration - elapsedTime;
        if (sleepTime > std::chrono::duration<double>::zero())
        {
            std::this_thread::sleep_for(sleepTime);
        }
        prevTime = std::chrono::high_resolution_clock::now();
    }
    context.reset();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext(imguiContext);
    glfwTerminate();

    return 0;
}

void OpenglViewer::onFrameBufferSizeChange(GLFWwindow *window, int width, int height)
{
    SPDLOG_INFO("framebuffer size changed ({} x {})", width, height);
    auto context = reinterpret_cast<Context*>(glfwGetWindowUserPointer(window));
    context->reshape(width, height);
}

void OpenglViewer::onKeyEvent(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
    SPDLOG_INFO("key: {}, scancode: {}, action: {}, mods: {}{}{}", key, scancode,
                action == GLFW_PRESS ? "Pressed" : action == GLFW_RELEASE ? "Released"
                                               : action == GLFW_REPEAT    ? "Repeat"
                                                                          : "Unknown",
                mods & GLFW_MOD_CONTROL ? "C" : "-",
                mods & GLFW_MOD_SHIFT ? "S" : "-",
                mods & GLFW_MOD_ALT ? "A" : "-");

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
    }
}

void OpenglViewer::onCursorPos(GLFWwindow* window, double x, double y)
{
    auto context = reinterpret_cast<Context*>(glfwGetWindowUserPointer(window));
    context->mouseMove(x, y);
}

void OpenglViewer::onMouseButton(GLFWwindow* window, int button, int action, int modifier)
{
    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, modifier);
    auto context = reinterpret_cast<Context*>(glfwGetWindowUserPointer(window));
    double x, y;
    glfwGetCursorPos(window, &x, &y);
    context->mouseButton(button, action, x, y);
}

void OpenglViewer::onCharacterEvent(GLFWwindow* window, unsigned int character)
{
    ImGui_ImplGlfw_CharCallback(window, character);
}

void OpenglViewer::onScroll(GLFWwindow* window, double xOffset, double yOffset)
{
    ImGui_ImplGlfw_ScrollCallback(window, xOffset, yOffset);
}