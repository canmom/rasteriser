#ifndef RASTERISER_MATERIAL_H
#define RASTERISER_MATERIAL_H

#include <string>
#include <iostream>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include "CImg.h"

class Material {
private:
	glm::vec3 diffuse_colour; //r, g, b albedo values
	bool has_texture;
	std::string diffuse_texture_file; //path to diffuse texture
	cimg_library::CImg<float> diffuse_texture;

public:
	Material(const glm::vec3 & dc) : diffuse_colour(dc), has_texture(false) {}
	Material(const glm::vec3 & dc, const std::string & dtf) : diffuse_colour(dc), has_texture(true), diffuse_texture_file(dtf), diffuse_texture(diffuse_texture_file.c_str()) {
		diffuse_texture.normalize(0.f,1.f);
	}
	glm::vec3 sample(const glm::vec2 & uv);
};

#endif