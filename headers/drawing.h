#ifndef RASTERISER_DRAWING_H
#define RASTERISER_DRAWING_H

#include <vector>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "CImg.h"

#include "light.h"
#include "arguments.h"
#include "face.h"
#include "material.h"

void draw_frame(const std::vector<glm::vec3>& model_vertices, const std::vector<Triangle>& faces,
    const std::vector<glm::vec3>&model_vertnormals, const std::vector<glm::vec2>& vertuvs, std::vector<Light>& lights, const std::vector<Material>& materials, const Args& arguments,
    cimg_library::CImg<unsigned char>* frame_buffer, cimg_library::CImg<float>* depth_buffer);

#endif //RASTERISER_DRAWING_H