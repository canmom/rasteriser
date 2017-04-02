project(SimpleRasteriser)
cmake_minimum_required(VERSION 2.8.12)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

add_executable(renderer renderer.cpp)
target_link_libraries(renderer ${CONAN_LIBS})
target_compile_features(renderer PRIVATE cxx_range_for)