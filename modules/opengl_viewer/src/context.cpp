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
    mVertexLayout = VertexLayout::create();

    std::shared_ptr<Shader> vertexShader = Shader::createFromFile(VERTEX_SHADER_PATH, GL_VERTEX_SHADER);
    std::shared_ptr<Shader> gnssFragmentShader = Shader::createFromFile(GNSS_FRAGMENT_SHADER_PATH, GL_FRAGMENT_SHADER);
    std::shared_ptr<Shader> filteredFragmentShader = Shader::createFromFile(FILTERED_FRAGMENT_SHADER_PATH, GL_FRAGMENT_SHADER);

    SPDLOG_INFO("vertex shader id: {}", vertexShader->get());
    SPDLOG_INFO("gnss fragment shader id: {}", gnssFragmentShader->get());
    SPDLOG_INFO("filtered fragment shader id: {}", filteredFragmentShader->get());
    if(!vertexShader || !gnssFragmentShader || !filteredFragmentShader)
    {
        return false;
    }
    mGnssProgram = Program::create({gnssFragmentShader, vertexShader});
    mFilteredProgram = Program::create({filteredFragmentShader, vertexShader});
    if(!mGnssProgram || !mFilteredProgram)
    {
        return false;
    }
    SPDLOG_INFO("program id: {}, {}", mGnssProgram->get(), mFilteredProgram->get());

    glClearColor(0.2f, 0.2f, 0.2f, 0.0f);

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
            mCameraPos = glm::vec3(0.0f, -1.5f, 13.0f);
            mCameraYaw = 0.0f;
            mCameraPitch = 0.0f;
        }
    }
    ImGui::End();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    mCameraFront = glm::rotate(glm::mat4(1.0f), glm::radians(mCameraYaw), glm::vec3(0.0f, 1.0f, 0.0f)) *
                   glm::rotate(glm::mat4(1.0f), glm::radians(mCameraPitch), glm::vec3(1.0f, 0.0f, 0.0f)) *
                   glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);
    auto view = glm::lookAt(mCameraPos, mCameraPos + mCameraFront, mCameraUp);
    auto projection = glm::perspective(glm::radians(45.0f), static_cast<float>(mWidth) / static_cast<float>(mHeight), 0.01f, 0.0f);

    auto pos = glm::vec3(0.0f, 0.0f, 0.0f);
    auto model = glm::translate(glm::mat4(1.0f), pos);
    auto transform = projection * view * model;

    auto gnssVertexBuffer = Buffer::createWithData(GL_ARRAY_BUFFER, GL_STREAM_DRAW, mGnssPoints.data(), sizeof(float) * mGnssPoints.size());
    mVertexLayout->setAttribute(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, 0);
    mGnssProgram->use();
    mGnssProgram->setUniform("transform", transform);
    glDrawArrays(GL_POINTS, 0, mGnssPoints.size() / 3);

    auto filteredVertexBuffer = Buffer::createWithData(GL_ARRAY_BUFFER, GL_STREAM_DRAW, mFilteredPoints.data(), sizeof(float) * mFilteredPoints.size());
    mVertexLayout->setAttribute(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, 0);
    mFilteredProgram->use();
    mFilteredProgram->setUniform("transform", transform);
    glDrawArrays(GL_POINTS, 0, mFilteredPoints.size() / 3);
}

void Context::addPoint(const Point& point)
{

    if(point.pointType == PointType::GNSS)
    {
        mGnssPoints.push_back(static_cast<float>(point.x / 100.0));
        mGnssPoints.push_back(static_cast<float>(point.y / 100.0));
        mGnssPoints.push_back(static_cast<float>(point.z / 100.0));
    }
    else
    {
        mFilteredPoints.push_back(static_cast<float>(point.x / 100.0));
        mFilteredPoints.push_back(static_cast<float>(point.y / 100.0));
        mFilteredPoints.push_back(static_cast<float>(point.z / 100.0));
    }
}