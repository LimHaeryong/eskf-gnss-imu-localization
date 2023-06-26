#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "opengl_viewer/common.h"

class Image
{
public:
    static std::unique_ptr<Image> load(const std::string& filename);
    static std::unique_ptr<Image> create(int width, int height, int channelCount = 4);
    ~Image();

    const uint8_t* getData() const {return mData;}
    int getWidth() const {return mWidth;}
    int getHeight() const {return mHeight;}
    int getChannelCount() const {return mChannelCount;}

    void setCheckImage(int gridX, int gridY);

private:
    Image() {}
    bool loadWidthStb(const std::string& filename);
    bool allocate(int width, int height, int channelCount);
    int mWidth = 0;
    int mHeight = 0;
    int mChannelCount = 0;
    uint8_t* mData = nullptr;
};

#endif // _IMAGE_H_