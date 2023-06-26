#include <imgui.h>

#include "opengl_viewer/context.h"
#include "opengl_viewer/image.h"

std::unique_ptr<Context> Context::create()
{
    auto context = std::unique_ptr<Context>(new Context());
    if(!context->init())
    {
        return nullptr;
    }
    return context;
}

bool Context::init()
{
    float vertices[] = {
        -0.5f, -0.5f, -0.5f, 0.0f, 0.0f,
        0.5f, -0.5f, -0.5f, 1.0f, 0.0f,
        0.5f,  0.5f, -0.5f, 1.0f, 1.0f,
        -0.5f,  0.5f, -0.5f, 0.0f, 1.0f,

        -0.5f, -0.5f,  0.5f, 0.0f, 0.0f,
        0.5f, -0.5f,  0.5f, 1.0f, 0.0f,
        0.5f,  0.5f,  0.5f, 1.0f, 1.0f,
        -0.5f,  0.5f,  0.5f, 0.0f, 1.0f,

        -0.5f,  0.5f,  0.5f, 1.0f, 0.0f,
        -0.5f,  0.5f, -0.5f, 1.0f, 1.0f,
        -0.5f, -0.5f, -0.5f, 0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f, 0.0f, 0.0f,

        0.5f,  0.5f,  0.5f, 1.0f, 0.0f,
        0.5f,  0.5f, -0.5f, 1.0f, 1.0f,
        0.5f, -0.5f, -0.5f, 0.0f, 1.0f,
        0.5f, -0.5f,  0.5f, 0.0f, 0.0f,

        -0.5f, -0.5f, -0.5f, 0.0f, 1.0f,
        0.5f, -0.5f, -0.5f, 1.0f, 1.0f,
        0.5f, -0.5f,  0.5f, 1.0f, 0.0f,
        -0.5f, -0.5f,  0.5f, 0.0f, 0.0f,

        -0.5f,  0.5f, -0.5f, 0.0f, 1.0f,
        0.5f,  0.5f, -0.5f, 1.0f, 1.0f,
        0.5f,  0.5f,  0.5f, 1.0f, 0.0f,
        -0.5f,  0.5f,  0.5f, 0.0f, 0.0f,
    };

    uint32_t indices[] = {
        0,  2,  1,  2,  0,  3,
        4,  5,  6,  6,  7,  4,
        8,  9, 10, 10, 11,  8,
        12, 14, 13, 14, 12, 15,
        16, 17, 18, 18, 19, 16,
        20, 22, 21, 22, 20, 23,
    };

    mVertexLayout = VertexLayout::create();
    mVertexBuffer = Buffer::createWithData(GL_ARRAY_BUFFER, GL_STATIC_DRAW, vertices, sizeof(float) * 120);
    mVertexLayout->setAttribute(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 5, 0);
    //mVertexLayout->setAttribute(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, sizeof(float) * 3);
    mVertexLayout->setAttribute(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 5, sizeof(float) * 3);
    mIndexBuffer = Buffer::createWithData(GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW, indices, sizeof(uint32_t) * 36);

    std::shared_ptr<Shader> vertexShader = Shader::createFromFile("/root/workspace/src/eskf_gnss_imu_localization/modules/opengl_viewer/shader/simple.vs", GL_VERTEX_SHADER);
    std::shared_ptr<Shader> fragmentShader = Shader::createFromFile("/root/workspace/src/eskf_gnss_imu_localization/modules/opengl_viewer/shader/simple.fs", GL_FRAGMENT_SHADER);
    SPDLOG_INFO("vertex shader id: {}", vertexShader->get());
    SPDLOG_INFO("fragment shader id: {}", fragmentShader->get());
    if(!vertexShader || !fragmentShader)
    {
        return false;
    }
    mProgram = Program::create({fragmentShader, vertexShader});
    if(!mProgram)
    {
        return false;
    }
    SPDLOG_INFO("program id: {}", mProgram->get());

    glClearColor(0.0f, 0.0f, 0.2f, 0.0f);

    auto image = Image::load("/root/workspace/src/eskf_gnss_imu_localization/modules/opengl_viewer/resources/container.jpg");
    if(!image)
    {
        return false;
    }
    SPDLOG_INFO("image: {}x{}, {} channels", image->getWidth(), image->getHeight(), image->getChannelCount());

    // // create check image
    // auto image = Image::create(512, 512);
    // image->setCheckImage(16, 16);

    auto image2 = Image::load("/root/workspace/src/eskf_gnss_imu_localization/modules/opengl_viewer/resources/awesomeface.png");
    if(!image2)
    {
        return false;
    }
    SPDLOG_INFO("image: {}x{}, {} channels", image2->getWidth(), image2->getHeight(), image2->getChannelCount());    

    mTexture = Texture::createFromImage(image.get());
    mTexture2 = Texture::createFromImage(image2.get());

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mTexture->get());
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, mTexture2->get());

    mProgram->use();
    mProgram->setUniform("tex", 0);
    mProgram->setUniform("tex2", 1);

    return true;
}

void Context::processInput(GLFWwindow* window)
{
    if(!mCameraControl)
    {
        return;
    }
    const float cameraSpeed = 0.05f;
    if(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    {
        mCameraPos += cameraSpeed * mCameraFront;
    }
    if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    {
        mCameraPos -= cameraSpeed * mCameraFront;
    }

    auto cameraRight = glm::normalize(glm::cross(mCameraUp, -mCameraFront));
    if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    {
        mCameraPos += cameraSpeed * cameraRight;
    }
    if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    {
        mCameraPos -= cameraSpeed * cameraRight;
    }

    auto cameraUp = glm::normalize(glm::cross(-mCameraFront, cameraRight));
    if(glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
    {
        mCameraPos += cameraSpeed * cameraUp;
    }
    if(glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
    {
        mCameraPos -= cameraSpeed * cameraUp;
    }

}

void Context::reshape(int32_t width, int32_t height)
{
    mWidth = width;
    mHeight = height;
    glViewport(0, 0, mWidth, mHeight);
}

void Context::mouseMove(double x, double y)
{
    if(!mCameraControl)
    {
        return;
    }
    auto pos = glm::vec2(static_cast<float>(x), static_cast<float>(y));
    auto deltaPos = pos - mPrevMousePos;

    const float cameraRotationSpeed = 0.8f;
    mCameraYaw -= deltaPos.x * cameraRotationSpeed;
    mCameraPitch -= deltaPos.y * cameraRotationSpeed;

    if(mCameraYaw < 0.0f)
    {
        mCameraYaw += 360.0f;
    }
    if(mCameraYaw > 360.0f)
    {
        mCameraYaw -= 360.0f;
    }

    if(mCameraPitch > 89.0f)
    {
        mCameraPitch = 89.0f;
    }
    if(mCameraPitch < -89.0f)
    {
        mCameraPitch = -89.0f;
    }

    mPrevMousePos = pos;
}

void Context::mouseButton(int button, int action, double x, double y)
{
    if(button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        if(action == GLFW_PRESS)
        {
            mPrevMousePos = glm::vec2(static_cast<float>(x), static_cast<float>(y));
            mCameraControl = true;
        }
        else if(action == GLFW_RELEASE)
        {
            mCameraControl = false;
        }
    }
}


void Context::render()
{
    if(ImGui::Begin("UI window"))
    {
        if(ImGui::ColorEdit4("clear color", glm::value_ptr(mClearColor)))
        {
            glClearColor(mClearColor.x, mClearColor.y, mClearColor.z, mClearColor.w);
        }
        ImGui::Separator();
        ImGui::DragFloat3("camera pos", glm::value_ptr(mCameraPos), 0.01f);
        ImGui::DragFloat("camera yaw", &mCameraYaw, 0.5f);
        ImGui::DragFloat("camera pitch", &mCameraPitch, 0.5f, -89.0f, 89.0f);
        ImGui::Separator();
        if(ImGui::Button("reset camera"))
        {
            mCameraPos = glm::vec3(0.0f, 0.0f, 3.0f);
            mCameraYaw = 0.0f;
            mCameraPitch = 0.0f;
        }
    }
    ImGui::End();

    std::vector<glm::vec3> cubePositions = {
        glm::vec3( 0.0f, 0.0f, 0.0f),
        glm::vec3( 2.0f, 5.0f, -15.0f),
        glm::vec3(-1.5f, -2.2f, -2.5f),
        glm::vec3(-3.8f, -2.0f, -12.3f),
        glm::vec3( 2.4f, -0.4f, -3.5f),
        glm::vec3(-1.7f, 3.0f, -7.5f),
        glm::vec3( 1.3f, -2.0f, -2.5f),
        glm::vec3( 1.5f, 2.0f, -2.5f),
        glm::vec3( 1.5f, 0.2f, -1.5f),
        glm::vec3(-1.3f, 1.0f, -1.5f),
    };

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    mProgram->use();

    mCameraFront = glm::rotate(glm::mat4(1.0f), glm::radians(mCameraYaw), glm::vec3(0.0f, 1.0f, 0.0f)) *
                   glm::rotate(glm::mat4(1.0f), glm::radians(mCameraPitch), glm::vec3(1.0f, 0.0f, 0.0f)) *
                   glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);

    auto view = glm::lookAt(mCameraPos, mCameraPos + mCameraFront, mCameraUp);

    auto projection = glm::perspective(glm::radians(45.0f), static_cast<float>(mWidth) / static_cast<float>(mHeight), 0.01f, 20.0f);
    
    for(size_t i = 0; i < cubePositions.size(); ++i)
    {
        auto& pos = cubePositions[i];
        auto model = glm::translate(glm::mat4(1.0f), pos);
        model = glm::rotate(model, glm::radians(static_cast<float>(glfwGetTime()) * 120.0f + 20.0f * static_cast<float>(i)), glm::vec3(1.0f, 0.5f, 0.0f));
        auto transform = projection * view * model;
        mProgram->setUniform("transform", transform);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
    }
}