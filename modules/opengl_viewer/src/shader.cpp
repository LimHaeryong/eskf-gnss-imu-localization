#include "opengl_viewer/shader.h"

Shader::~Shader()
{
    if(mShader)
    {
        glDeleteShader(mShader);
    }
}

std::unique_ptr<Shader> Shader::createFromFile(const std::string& filename, GLenum shaderType)
{
    auto shader = std::unique_ptr<Shader>(new Shader());
    if(!shader->loadFile(filename, shaderType))
    {
        return nullptr;
    }
    return shader;
}

bool Shader::loadFile(const std::string& filename, GLenum shaderType)
{
    auto text = loadTextFile(filename);
    if(text.empty())
    {
        return false;
    }
    
    auto& code = text;
    const char* codePtr = code.c_str();
    int32_t codeLength = static_cast<int32_t>(code.length());

    mShader = glCreateShader(shaderType);
    glShaderSource(mShader, 1, (const GLchar* const*)&codePtr, &codeLength);
    glCompileShader(mShader);

    int success = 0;
    glGetShaderiv(mShader, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        char infoLog[1024];
        glGetShaderInfoLog(mShader, 1024, nullptr, infoLog);
        SPDLOG_ERROR("failed to compile shader: \"{}\"", filename);
        SPDLOG_ERROR("reason: {}", infoLog);
        return false;
    }
    return true;
}