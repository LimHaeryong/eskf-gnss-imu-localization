#ifndef _TEXTURE_H_
#define _TEXTURE_H_

#include "opengl_viewer/common.h"
#include "opengl_viewer/image.h"

class Texture
{
public:
    static std::unique_ptr<Texture> createFromImage(const Image* image);
    ~Texture();

    uint32_t get() const {return mTexture;}
    void bind() const;
    void setFilter(uint32_t minFilter, uint32_t magFilter) const;
    void setWrap(uint32_t sWrap, uint32_t tWrap) const;
private:
    Texture() {}
    void createTexture();
    void setTextureFromImage(const Image* image);

    uint32_t mTexture = 0;
};

#endif // _TEXTURE_H_