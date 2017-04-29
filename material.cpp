#include "material.h"

#include <stdexcept>
#include <string>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

using glm::vec2;
using glm::vec3;

vec3 Material::sample(const vec2 & uv) const {
	if (has_texture) {
		// if(uv.x > 1.f or uv.x < 0.f) {
		// 	throw std::out_of_range("Texture sampling error: U-coordinate out of range: " + std::to_string(uv.x));
		// }
		// if(uv.y > 1.f or uv.y < 0.f) {
		// 	throw std::out_of_range("Texture sampling error: V-coordinate out of range: " + std::to_string(uv.y));
		// }
		float u = uv.x * diffuse_texture.width();
		float v = (1.f-uv.y) * diffuse_texture.height();
		return vec3(diffuse_texture.linear_atXY(u,v,0,0),diffuse_texture.linear_atXY(u,v,0,1),diffuse_texture.linear_atXY(u,v,0,2));
	}
	else {
		return diffuse_colour;
	}
}