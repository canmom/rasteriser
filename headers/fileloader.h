#ifndef RASTERISER_FILELOADER_H
#define RASTERISER_FILELOADER_H

#include <vector>
#include <string>

#include <glm/vec3.hpp>

void load_obj(std::string file, std::vector<glm::vec3> &vertices, std::vector<glm::uvec3> &faces);

#endif //RASTERISER_FILELOADER_H