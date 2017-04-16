#ifndef RASTERISER_SHADING_H
#define RASTERISER_SHADING_H

#include <vector>

#include <glm/vec3.hpp>

#include "light.h"

float light_contribution(const glm::vec3& normal, float albedo, const Light& light);

unsigned char shade(const glm::vec3& normal, float albedo, const std::vector<Light> lights);

#endif //RASTERISER_SHADING_H