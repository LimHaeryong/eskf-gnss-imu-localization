#include "opengl_viewer/image.h"

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

std::unique_ptr<Image> Image::load(const std::string& filename)
{
    auto image = std::unique_ptr<Image>(new Image());
    if(!image->loadWidthStb(filename))
    {
        return nullptr;
    }
    return image;
}

std::unique_ptr<Image> Image::create(int width, int height, int channelCount)
{
    auto image = std::unique_ptr<Image>(new Image());
    if(!image->allocate(width, height, channelCount))
    {
        return nullptr;
    }
    return image;
}

Image::~Image()
{
    if(mData)
    {
        stbi_image_free(mData);
    }
}

void Image::setCheckImage(int gridX, int gridY)
{
    for(int j = 0; j < mHeight; ++j)
    {
        for(int i = 0; i < mWidth; ++i)
        {
            int pos = (j * mWidth  + i) * mChannelCount;
            bool even = ((i / gridX) + (j / gridY)) % 2 == 0;
            uint8_t value = even ? 255 : 0;
            for(int k = 0; k < mChannelCount; ++k)
            {
                mData[pos + k] = value;
            }
            if(mChannelCount > 3)
            {
                mData[pos + 3] = 255;
            }
        }
    }
}

bool Image::loadWidthStb(const std::string& filename)
{
    stbi_set_flip_vertically_on_load(true);
    mData = stbi_load(filename.c_str(), &mWidth, &mHeight, &mChannelCount, 0);
    if(!mData)
    {
        SPDLOG_ERROR("failed to load image: {}", filename);
        return false;
    }
    return true;
}

bool Image::allocate(int width, int height, int channelCount)
{
    mWidth = width;
    mHeight = height;
    mChannelCount = channelCount;
    mData = (uint8_t*)malloc(mWidth * mHeight * mChannelCount);
    return mData ? true : false;
}