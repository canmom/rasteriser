#ifndef RASTERISER_SWIZZLE_H
#define RASTERISER_SWIZZLE_H

//limited subset of as-needed swizzle functions to avoid using full GLM set

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

glm::vec2 xy(const glm::vec3& v);

glm::vec3 xyz(const glm::vec4& v);

#endif //RASTERISER_SWIZZLE_H