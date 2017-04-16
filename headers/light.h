#ifndef RASTERISER_LIGHT_H
#define RASTERISER_LIGHT_H

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

struct Light {
    glm::vec3 direction;
    float intensity;
    glm::vec3 colour;
    glm::vec3 trans_dir;
    Light(glm::vec3 d, float i, glm::vec3 c) : direction(d), intensity(i), colour(c) { }
    void transform(const glm::mat4& transformation);
};

#endif //RASTERISER_LIGHT_H