#ifndef _BUFFER_H_
#define _BUFFER_H_

#include "opengl_viewer/common.h"

class Buffer
{
public:
    static std::unique_ptr<Buffer> createWithData(uint32_t bufferType, uint32_t usage, const void* data, size_t dataSize);

    ~Buffer();
    uint32_t get() const {return mBuffer;}
    void bind() const;
private:
    Buffer() {}
    bool init(uint32_t bufferType, uint32_t usage, const void* data, size_t dataSize);

    uint32_t mBuffer = 0;
    uint32_t mBufferType = 0;
    uint32_t mUsage = 0;

};

#endif // _BUFFER_H_