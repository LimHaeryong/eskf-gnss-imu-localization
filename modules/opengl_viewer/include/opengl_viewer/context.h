#ifndef _CONTEXT_H_
#define _CONTEXT_H_

#include "opengl_viewer/type.h"
#include "opengl_viewer/common.h"
#include "opengl_viewer/shader.h"
#include "opengl_viewer/program.h"
#include "opengl_viewer/buffer.h"
#include "opengl_viewer/vertex_layout.h"
#include "opengl_viewer/texture.h"

class Context
{
public:
    static std::unique_ptr<Context> create();
    void render();
    void processInput(GLFWwindow* window);
    void reshape(int32_t width, int32_t height);
    void mouseMove(double x, double y);
    void mouseButton(int button, int action, double x, double y);

    void addPoint(const Point& point);

private:
    Context() {}
    bool init();
    std::unique_ptr<Program> mProgram;

    std::unique_ptr<VertexLayout> mVertexLayout;
    std::unique_ptr<Buffer> mVertexBuffer;


    bool mCameraControl = false;
    glm::vec2 mPrevMousePos = glm::vec2(0.0f);
    float mCameraPitch = 0.0f;
    float mCameraYaw = 0.0f;
    glm::vec3 mCameraPos = glm::vec3(0.0f, -1.5f, 13.0f);
    glm::vec3 mCameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 mCameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

    int32_t mWidth = WINDOW_WIDTH;
    int32_t mHeight = WINDOW_HEIGHT;

    glm::vec4 mClearColor = glm::vec4(0.2f, 0.2f, 0.2f, 0.0f);

    std::vector<Point> mPoints;
};

#endif // _CONTEXT_H_