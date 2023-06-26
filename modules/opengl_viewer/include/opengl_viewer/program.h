#ifndef _PROGRAM_H_
#define _PROGRAM_H_

#include "opengl_viewer/common.h"
#include "opengl_viewer/shader.h"

class Program
{
public:
    static std::unique_ptr<Program> create(const std::vector<std::shared_ptr<Shader>>& shaders);
    ~Program();
    uint32_t get() const {return mProgram;}
    void use() const;

    void setUniform(const std::string& name, int value) const;
    void setUniform(const std::string& name, const glm::mat4& value) const;

private:
    Program() {}
    bool link(const std::vector<std::shared_ptr<Shader>>& shaders);
    uint32_t mProgram = 0;
};

#endif // _PROGRAM_H_