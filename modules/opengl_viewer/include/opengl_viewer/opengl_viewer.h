#ifndef _OPENGL_VIEWER_H_
#define _OPENGL_VIEWER_H_

#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "opengl_viewer/type.h"
#include "opengl_viewer/context.h"
#include "opengl_viewer/threadsafe_queue.hpp"

class OpenglViewer
{
public:
    using QueueSharedPtr = std::shared_ptr<ThreadsafeQueue<std::shared_ptr<Point>>>;

    OpenglViewer();
    int run();
    QueueSharedPtr getQueue() const { return mPointQueue; }

private:
    static void onFrameBufferSizeChange(GLFWwindow *window, int width, int height);
    static void onKeyEvent(GLFWwindow *window, int key, int scancode, int action, int mods);
    static void onCursorPos(GLFWwindow* window, double x, double y);
    static void onMouseButton(GLFWwindow* window, int button, int action, int modifier);
    static void onCharacterEvent(GLFWwindow* window, unsigned int character);
    static void onScroll(GLFWwindow* window, double xOffset, double yOffset);

    QueueSharedPtr mPointQueue;
};

#endif // _OPENGL_VIEWER_H_