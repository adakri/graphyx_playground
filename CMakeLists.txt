# Cmake minimum version
cmake_minimum_required(VERSION 3.21)

# Set cmake cuda (if it does not exist)
set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.1/bin/")
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.1")


project(graphyx_playground LANGUAGES CXX C CUDA)

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)

# C++ std
set(CMAKE_CXX_STANDARD 17)


# Set output folder/ build folder
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "../bin/")
set(CMAKE_BINARY_DIR ${CMAKE_BUILD_DIR})

set(PROJECT_ROOT_PATH ${PROJECT_SOURCE_DIR})   

# All sources
file(GLOB SRCS 
    "src/*.cpp"
    "src/Boids/*.cpp"
    "src/utils/*.cpp"
    "kernels/*.cu"
)

# Add imgui files
#set(IMGUI_SRCS "${PROJECT_SOURCE_DIR}/external/imgui/src")
#list(APPEND SRCS "${IMGUI_SRC}/imgui_demo.cpp"
#                 "${IMGUI_SRC}/imgui_draw.cpp"
#                 "${IMGUI_SRC}/imgui_impl_glfw.cpp"
#                 "${IMGUI_SRC}/imgui_impl_opengl3.cpp"
#                 "${IMGUI_SRC}/imgui_widgets.cpp"
#                 "${IMGUI_SRC}/imgui.cpp")


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

find_package(OpenGL REQUIRED)
#find_package(CUDA REQUIRED)
add_definitions(${OPENGL_DEFINITIONS})
find_package(X11 REQUIRED)
find_package(glm REQUIRED)
find_package(glfw3 REQUIRED)

# note that the order is important for setting the libs
# use pkg-config --libs $(pkg-config --print-requires --print-requires-private glfw3) in a terminal to confirm
set(LIBS glfw X11 Xrandr Xinerama Xi Xxf86vm Xcursor GL dl pthread freetype ${ASSIMP_LIBRARY})
# Relevant static libraries required
add_library(GLAD "src/glad.c")
set(LIBS ${LIBS} GLAD)

message((${LIBS}))

# Add exec
add_executable(graphyx_playground ${SRCS})
target_link_libraries(graphyx_playground ${LIBS})

# CUDA 12.x has dropped support for Kepler compute 3.x devices.
set_target_properties(graphyx_playground PROPERTIES CUDA_ARCHITECTURES "61;72")

include_directories(${CMAKE_SOURCE_DIR}/external ${CMAKE_SOURCE_DIR}/include )

