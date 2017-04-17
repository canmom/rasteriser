#ifndef RASTERISER_ARGUMENTS_H
#define RASTERISER_ARGUMENTS_H

#include <string>

struct Args {
    unsigned int image_width;
    unsigned int image_height;
    float aspect_ratio;
    float angle;
    bool spin;
    std::string obj_file;
    std::string lights_file;
    Args(int argc, char** argv);
};

#endif