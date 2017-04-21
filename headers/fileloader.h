#ifndef RASTERISER_FILELOADER_H
#define RASTERISER_FILELOADER_H

#include <vector>
#include <string>

#include <glm/vec3.hpp>

#include "light.h"
#include "face.h"

void load_obj(std::string file, std::vector<glm::vec3> &vertices, std::vector<Triangle> &faces, std::vector<glm::vec3> &vertnormals);

void load_lights(std::string file, std::vector<Light> &lights);

#endif //RASTERISER_FILELOADER_H