#include "material.h"

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

using glm::vec2;
using glm::vec3;

vec3 Material::sample(const vec2 & uv) const {
	if (has_texture) {
		float xcoord = uv.x * diffuse_texture.width();
		float ycoord = uv.y * diffuse_texture.height();
		return vec3(diffuse_texture.linear_atXY(xcoord,ycoord,0,0),diffuse_texture.linear_atXY(xcoord,ycoord,0,1),diffuse_texture.linear_atXY(xcoord,ycoord,0,2));
	}
	else {
		return diffuse_colour;
	}
}