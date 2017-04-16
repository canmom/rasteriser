project(SimpleRasteriser)
cmake_minimum_required(VERSION 2.8.12)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

add_library(geometry geometry.cpp)
target_compile_features(geometry PRIVATE cxx_range_for)

add_executable(renderer renderer.cpp)
target_link_libraries(renderer ${CONAN_LIBS} -lX11 -lpthread geometry)
target_compile_features(renderer PRIVATE cxx_range_for)