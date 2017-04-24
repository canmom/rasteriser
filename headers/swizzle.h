#ifndef RASTERISER_SWIZZLE_H
#define RASTERISER_SWIZZLE_H

//limited subset of as-needed swizzle functions to avoid using full GLM set

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

inline glm::vec2 xy(const glm::vec3& v) {
    //return 2D vector containing the first two components of v

    return glm::vec2(v.x,v.y);
}

inline glm::vec3 xyz(const glm::vec4& v) {
	//return 3D vector containing the first three comments of v

    return glm::vec3(v.x,v.y,v.z);
}

#endif //RASTERISER_SWIZZLE_H