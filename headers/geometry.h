#ifndef RASTERISER_GEOMETRY_H
#define RASTERISER_GEOMETRY_H

#include <vector>
#include <array>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "light.h"

glm::mat4 transformation_matrix(float scale, const glm::vec3 & displacement, const glm::vec3 & tait_bryan);

glm::mat4 camera_matrix(const glm::mat4& modelview,float aspect_ratio);

glm::vec3 transform_direction(const glm::mat4& transformation, const glm::vec3& point);

glm::vec4 transform_point(const glm::mat4& transformation, const glm::vec3& point);

glm::vec4 z_divide(const glm::vec4& clip_vertex);

float remap_ndc(float value, float high);

glm::vec4 ndc_to_raster(int width,int height,const glm::vec4& ndc_vertex);

float signed_area_2d(const std::array<glm::vec4,3> & v);

void transform_vertices(const glm::mat4& transformation, const std::vector<glm::vec3>& vertices, std::vector<glm::vec4>& result);

void transform_normals(const glm::mat4& transformation, const std::vector<glm::vec3>& vertices, std::vector<glm::vec3>& result);

void z_divide_all(const std::vector<glm::vec4>& clip_vertices, std::vector<glm::vec4>& ndc_vertices);

void xyz_all(const std::vector<glm::vec4>& homo_vertices, std::vector<glm::vec3>& cart_vertices);

void transform_lights(const glm::mat4& transformation, std::vector<Light>& lights);

void ndc_to_raster_all(int width, int height, const std::vector<glm::vec4>& ndc_vertices, std::vector<glm::vec4> & raster_vertices);

#endif