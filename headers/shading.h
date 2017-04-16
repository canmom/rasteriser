#ifndef RASTERISER_SHADING_H
#define RASTERISER_SHADING_H

#include <vector>

#include <glm/vec3.hpp>

#include "light.h"

glm::vec3 light_contribution(const glm::vec3& normal, const glm::vec3& albedo, const Light& light);

glm::uvec3 shade(const glm::vec3& normal, const glm::vec3& albedo, const std::vector<Light> lights);

#endif //RASTERISER_SHADING_H