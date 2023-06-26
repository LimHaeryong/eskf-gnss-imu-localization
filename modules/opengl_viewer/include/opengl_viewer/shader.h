#ifndef _SHADER_H_
#define _SHADER_H_

#include "opengl_viewer/common.h"

class Shader
{
public:
    // 객체를 생성하려면 createFromFile 함수를 이용할 수 밖에 없음.
    static std::unique_ptr<Shader> createFromFile(const std::string& filename, GLenum shaderType);
    ~Shader();
    uint32_t get() const {return mShader;}
private:
    Shader() {}
    bool loadFile(const std::string& filename, GLenum shaderType);
    uint32_t mShader = 0;
};

#endif // _SHADER_H_