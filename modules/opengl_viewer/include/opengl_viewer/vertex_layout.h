#ifndef _VERTEX_LAYOUT_H_
#define _VERTEX_LAYOUT_H_

#include "opengl_viewer/common.h"

class VertexLayout
{
public:
    static std::unique_ptr<VertexLayout> create();
    ~VertexLayout();

    uint32_t get() const { return mVertexArrayObject; }
    void bind() const;
    void setAttribute(uint32_t attribIndex, int count, uint32_t type, bool normalized, size_t stride, uint64_t offset) const;
    void disableAttribute(int attribIndex) const;

private:
    VertexLayout() {}
    void init();

    uint32_t mVertexArrayObject = 0;
};

#endif // _VERTEX_LAYOUT_H_