project(SimpleRasteriser)
cmake_minimum_required(VERSION 2.8.12)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(headers vendor/cimg vendor/tinyobjloader)

add_library(geometry geometry.cpp)
add_library(shading shading.cpp)
add_library(drawing drawing.cpp)

add_executable(renderer renderer.cpp)
target_link_libraries(drawing shading)
target_link_libraries(renderer ${CONAN_LIBS} -lX11 -lpthread geometry shading drawing)