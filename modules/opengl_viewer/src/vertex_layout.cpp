#include "opengl_viewer/vertex_layout.h"

std::unique_ptr<VertexLayout> VertexLayout::create()
{
    auto vertexLayout = std::unique_ptr<VertexLayout>(new VertexLayout());
    vertexLayout->init();
    return vertexLayout;
}

VertexLayout::~VertexLayout()
{
    if(mVertexArrayObject)
    {
        glDeleteVertexArrays(1, &mVertexArrayObject);
    }
}

void VertexLayout::bind() const
{
    glBindVertexArray(mVertexArrayObject);
}

void VertexLayout::setAttribute(uint32_t attribIndex, int count, uint32_t type, bool normalized, size_t stride, uint64_t offset) const
{
    glEnableVertexAttribArray(attribIndex);
    glVertexAttribPointer(attribIndex, count, type, normalized, stride, (const void*)offset);
}

void VertexLayout::init()
{
    glGenVertexArrays(1, &mVertexArrayObject);
    bind();
}