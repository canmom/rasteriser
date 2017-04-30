#ifndef RASTERISER_ARGUMENTS_H
#define RASTERISER_ARGUMENTS_H

#include <string>
#include <glm/vec3.hpp>

struct Args {
    unsigned int image_width;
    unsigned int image_height;
    float aspect_ratio;
    bool spin;
    bool flat;
    bool wind_clockwise;
    float scale;
    glm::vec3 displacement;
    glm::vec3 tait_bryan_angles;
    std::string obj_file;
    std::string lights_file;
    std::string materials_directory;
    Args(int argc, char** argv);
};

#endif