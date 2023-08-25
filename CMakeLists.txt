cmake_minimum_required(VERSION 3.0.0)
project(motion_graph VERSION 0.1.0)
cmake_policy(SET CMP0072 NEW)

# Always Pu
find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)
find_package(GLUT REQUIRED)

add_executable(motion_graph 
    src/main.cpp
    src/bvh_motion.cpp
    src/bvh_motion.h
    src/parse_bvh.cpp
    src/parse_bvh.h
    src/joint.cpp
    src/joint.h
    src/motion_graph.cpp
    src/motion_graph.h
    #src/stb_image.h
    )

add_subdirectory(dependencies/imgui/)

target_include_directories(motion_graph
    PRIVATE dependencies/imgui)

target_link_directories(motion_graph BEFORE
    PUBLIC dependencies/imgui/
    )

target_link_libraries(motion_graph 
    glfw
    GLUT::GLUT
    OpenGL::GL
    OpenGL::GLU
    glm::glm
    imgui
    #stb_image
    )