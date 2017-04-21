project(SimpleRasteriser)
cmake_minimum_required(VERSION 2.8.12)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(headers vendor/cimg vendor/tinyobjloader vendor/fastcppcsvparser)

add_library(swizzle swizzle.cpp)
add_library(geometry geometry.cpp)
add_library(shading shading.cpp)
add_library(drawing drawing.cpp)
add_library(fileloader fileloader.cpp)
add_library(args arguments.cpp)

add_executable(renderer renderer.cpp)
target_link_libraries(geometry swizzle)
target_link_libraries(drawing shading geometry swizzle)
target_link_libraries(renderer ${CONAN_LIBS} geometry shading drawing fileloader args)

if(UNIX AND NOT APPLE)
	#if building under linux
	target_link_libraries(renderer -lx11 -lpthread)
endif()