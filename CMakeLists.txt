project(SimpleRasteriser)
cmake_minimum_required(VERSION 2.8.12)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(headers vendor/cimg vendor/tinyobjloader vendor/fastcppcsvparser)

add_executable(renderer renderer.cpp material.cpp geometry.cpp shading.cpp drawing.cpp fileloader.cpp arguments.cpp)
target_link_libraries(renderer ${CONAN_LIBS})

if(UNIX AND NOT APPLE)
	#if building under linux
	target_link_libraries(renderer -lX11 -lpthread)
endif()