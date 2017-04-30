#ifndef RASTERISER_FILELOADER_H
#define RASTERISER_FILELOADER_H

#include <vector>
#include <string>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "tiny_obj_loader.h"

#include "light.h"
#include "face.h"
#include "material.h"
#include "arguments.h"

void load_obj(const Args & arguments, std::vector<glm::vec3> &vertices, std::vector<Triangle> &triangles, std::vector<glm::vec3> & vertnormals, std::vector<glm::vec2>& vertuvs, std::vector<Material>& materials);

void load_lights(std::string file, std::vector<Light> &lights);

#endif //RASTERISER_FILELOADER_H