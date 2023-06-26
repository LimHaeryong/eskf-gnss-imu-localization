#ifndef _COMMON_H_
#define _COMMON_H_

#include <memory>
#include <string>
#include <optional>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>

std::string loadTextFile(const std::string& filename);

#endif // _COMMON_H_