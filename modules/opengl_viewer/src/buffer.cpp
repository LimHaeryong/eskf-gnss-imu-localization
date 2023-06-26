#include "opengl_viewer/buffer.h"

std::unique_ptr<Buffer> Buffer::createWithData(uint32_t bufferType, uint32_t usage, const void* data, size_t dataSize)
{
    auto buffer = std::unique_ptr<Buffer>(new Buffer());
    if(!buffer->init(bufferType, usage, data, dataSize))
    {
        return nullptr;
    }
    return buffer;
}

Buffer::~Buffer()
{
    if(mBuffer)
    {
        glDeleteBuffers(1, &mBuffer);
    }
}

void Buffer::bind() const
{
    glBindBuffer(mBufferType, mBuffer);
}

bool Buffer::init(uint32_t bufferType, uint32_t usage, const void* data, size_t dataSize)
{
    mBufferType = bufferType;
    mUsage = usage;
    glGenBuffers(1, &mBuffer);
    bind();
    glBufferData(mBufferType, dataSize, data, usage);
    return true;
}