cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME opengl_viewer)
project(${PROJECT_NAME} LANGUAGES CXX)

include(Dependency.cmake)

add_library(${PROJECT_NAME}
    src/opengl_viewer.cpp
    src/common.cpp
    src/shader.cpp
    src/program.cpp
    src/context.cpp
    src/buffer.cpp
    src/vertex_layout.cpp
    src/image.cpp
    src/texture.cpp
    include/opengl_viewer/opengl_viewer.h
    include/opengl_viewer/common.h
    include/opengl_viewer/shader.h
    include/opengl_viewer/program.h
    include/opengl_viewer/context.h
    include/opengl_viewer/buffer.h
    include/opengl_viewer/vertex_layout.h
    include/opengl_viewer/image.h
    include/opengl_viewer/texture.h
)
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${DEP_INCLUDE_DIR}
)

target_link_directories(${PROJECT_NAME} PUBLIC
    ${DEP_LIB_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${DEP_LIBS}
) 

set(WINDOW_WIDTH 960)
set(WINDOW_HEIGHT 540)
set(VERTEX_SHADER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader.vs)
set(FRAGMENT_SHADER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader.fs)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    WINDOW_WIDTH=${WINDOW_WIDTH}
    WINDOW_HEIGHT=${WINDOW_HEIGHT}
    VERTEX_SHADER_PATH="${VERTEX_SHADER_PATH}"
    FRAGMENT_SHADER_PATH="${FRAGMENT_SHADER_PATH}"
)
add_dependencies(${PROJECT_NAME}
    ${DEP_LIST}
)