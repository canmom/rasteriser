#ifndef RASTERISER_SWIZZLE_H
#define RASTERISER_SWIZZLE_H

//limited subset of as-needed swizzle functions to avoid using full GLM set

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

glm::vec2 xy(const glm::vec3& v) {
    //return 2D vector containing the first two components of v

    return glm::vec2(v.x,v.y);
}

#endif //RASTERISER_SWIZZLE_H