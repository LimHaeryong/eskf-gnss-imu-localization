#include "opengl_viewer/common.h"

#include <fstream>
#include <sstream>

std::string loadTextFile(const std::string& filename)
{
    std::ifstream fin(filename);
    if(!fin.is_open())
    {
        SPDLOG_ERROR("failed to open file: {}", filename);
        return std::string();
    }
    std::stringstream text;
    text << fin.rdbuf();
    return text.str();
}