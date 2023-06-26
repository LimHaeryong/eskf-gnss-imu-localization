#include "opengl_viewer/program.h"

std::unique_ptr<Program> Program::create(const std::vector<std::shared_ptr<Shader>> &shaders)
{
    auto program = std::unique_ptr<Program>(new Program());
    if (!program->link(shaders))
    {
        return nullptr;
    }
    return program;
}

Program::~Program()
{
    if (mProgram)
    {
        glDeleteProgram(mProgram);
    }
}

bool Program::link(const std::vector<std::shared_ptr<Shader>> &shaders)
{
    mProgram = glCreateProgram();
    for (auto &shader : shaders)
    {
        glAttachShader(mProgram, shader->get());
    }
    glLinkProgram(mProgram);

    int success = 0;
    glGetProgramiv(mProgram, GL_LINK_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetProgramInfoLog(mProgram, 1024, nullptr, infoLog);
        SPDLOG_ERROR("failed to link program: {}", infoLog);
        return false;
    }
    return true;
}

void Program::use() const
{
    glUseProgram(mProgram);
}

void Program::setUniform(const std::string &name, int value) const
{
    auto location = glGetUniformLocation(mProgram, name.c_str());
    glUniform1i(location, value);
}

void Program::setUniform(const std::string &name, const glm::mat4 &value) const
{
    auto location = glGetUniformLocation(mProgram, name.c_str());
    glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(value));
}