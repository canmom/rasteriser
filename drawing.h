#ifndef RASTERISER_DRAWING_H
#define RASTERISER_DRAWING_H

#include <vector>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "./vendor/cimg/CImg.h"
#include "light.h"

float edge(const glm::vec2& point, const glm::vec3& vert1, const glm::vec3& vert2);

glm::vec3 barycentric(const glm::vec2& point, const glm::vec3& vert0, const glm::vec3& vert1, const glm::vec3& vert2);

template <typename T>
inline T interpolate(T v0, T v1, T v2, const glm::vec3& bary);

void bounding_box(glm::uvec2& top_left, glm::uvec2& bottom_right,
    const glm::vec3& vert0, const glm::vec3& vert1, const glm::vec3& vert2,
    unsigned int image_width, unsigned int image_height);

void update_pixel(unsigned int raster_x, unsigned int raster_y,
    const glm::vec3& vert0, const glm::vec3& vert1, const glm::vec3& vert2, const glm::vec3& normal, const std::vector<Light>& lights,
    cimg_library::CImg<unsigned char>& frame_buffer, cimg_library::CImg<float>& depth_buffer);

void draw_triangle(const glm::uvec3& face, const std::vector<glm::vec3>& raster_vertices, const std::vector<glm::vec3>& camera_vertices, const std::vector<Light>& lights,
    cimg_library::CImg<unsigned char>* frame_buffer, cimg_library::CImg<float>* depth_buffer,
    unsigned int image_width, unsigned int image_height);

#endif //RASTERISER_DRAWING_H