#include <vector>
#include <array>
#include <algorithm>
#include <functional>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "light.h"
#include "geometry.h"
#include "swizzle.h"

using std::vector;
using std::array;
using glm::vec3;
using glm::vec4;
using glm::mat4;
using namespace std::placeholders;

mat4 transformation_matrix(float factor, const vec3 & displacement, const vec3 & tait_bryan) {
    //create a matrix for scaling an object by factor relative to origin, rotating it by YXZ Tait-Bryan angles tait_bryan, and displacing it by displacement
    return glm::translate(mat4(1.f),displacement) * glm::rotate(mat4(1.f),tait_bryan.y,vec3(0.f,1.f,0.f)) * glm::rotate(mat4(1.f),tait_bryan.x,vec3(1.f,0.f,0.f)) * glm::rotate(mat4(1.f),tait_bryan.z,vec3(0.f,0.f,1.f)) * glm::scale(mat4(1.f),vec3(factor));
}

mat4 camera_matrix(const mat4& modelview,float aspect_ratio) {
    //premultiply a transformation matrix by a perspective projection matrix to make a camera matrix
    
    mat4 perspective = glm::perspective(glm::radians(45.0f),aspect_ratio,0.1f,6.f);

    return perspective * modelview;
}

vec3 transform_direction(const mat4& transformation, const vec3& point) {
    //transform a 3D vector (implicitly in homogeneous coordinates with w=0) using a 4x4 matrix
    //and return a 3D vector, ASSUMING TRANSFORMATION DOES NOT MODIFY W

    vec4 homo_point(point.x,point.y,point.z,0.f);
    vec4 thp = transformation * homo_point;
    return vec3(thp.x,thp.y,thp.z);
}

vec4 transform_point(const mat4& transformation, const vec3& point) {
    //transform a 3D vector (implicitly in homogeneous coordinates with w=1) using a 4x4 matrix
    //and return as a 4-vector in homogeneous coordinates

    vec4 homo_point(point.x,point.y,point.z,1.f);
    return transformation * homo_point;
}

vec4 z_divide(const vec4& clip_vertex) {
    //take 4D vector clip_vertex in homogeneous coordinates and return equivalent 3D vector
    return vec4(
        clip_vertex.x/clip_vertex.w,
        clip_vertex.y/clip_vertex.w,
        clip_vertex.z/clip_vertex.w,
        1.f/clip_vertex.w
    );
}

float remap_ndc(float value, float high) {
    //scale a quantity defined in the range [-1,1] to the range [0,high]
    return 0.5f*(value + 1.0f)*high;
}

vec4 ndc_to_raster(int width,int height,const vec4& ndc_vertex) {
    //return vertex with X and Y coordinates scaled from Normalised Device Coordinates to an image plane with given width and height
    return vec4(
        remap_ndc(ndc_vertex.x,width),
        remap_ndc(-ndc_vertex.y,height),
        ndc_vertex.z,
        ndc_vertex.w);
}

float signed_area_2d(const array<vec4,3>& v) {
    //calculate the signed area of a triangle in the xy plane
    //the arguments are vec4 not vec2 for convenience, but only xy coordinates are relevant 
    return -0.5f*(
        v[0].x * v[1].y - v[1].x * v[0].y +
        v[1].x * v[2].y - v[2].x * v[1].y +
        v[2].x * v[0].y - v[0].x * v[2].y);
}

void transform_vertices(const mat4& transformation, const vector<vec3>& vertices, vector<vec4>& result) {
    //Apply transformation to each vec3 in vertices and store results in result

    result.resize(vertices.size());

    //partially apply the transform_point function with the given transformation matrix
    auto tp = std::bind(transform_point, transformation, _1);
    
    //apply the transformation to every point and store the results in 'result'
    std::transform(vertices.begin(),vertices.end(),result.begin(),tp);
}

void transform_normals(const mat4& transformation, const vector<vec3>& vertices, vector<vec3>& result) {
    //Apply transformation to each vec3 in vertices and store results in result

    result.resize(vertices.size());
    mat4 in_tp_transformation = glm::transpose(glm::inverse(transformation)); //for transforming normals

    //partially apply the transform_point function with the given transformation matrix
    auto td = std::bind(transform_direction, in_tp_transformation, _1);
    
    //apply the transformation to every point and store the results in 'result'
    std::transform(vertices.begin(),vertices.end(),result.begin(),td);
}

void z_divide_all(const vector<vec4>& clip_vertices, vector<vec4>& ndc_vertices) {
    //apply z_devide to all vertices in clip_vertices, and store result in ndc_vertices

    ndc_vertices.resize(clip_vertices.size());

    std::transform(clip_vertices.begin(),clip_vertices.end(),ndc_vertices.begin(),z_divide);
}

void xyz_all(const std::vector<glm::vec4>& homo_vertices, std::vector<glm::vec3>& cart_vertices) {
    cart_vertices.resize(homo_vertices.size());

    std::transform(homo_vertices.begin(),homo_vertices.end(),cart_vertices.begin(),xyz);
}

void Light::transform(const mat4& transformation) {
    //set the trans_dir member of a Light to the world-space direction multiplied by the matrix transformation
    trans_dir = glm::normalize(transform_direction(transformation,direction));
}

void transform_lights(const mat4& transformation, vector<Light>& lights) {
    for (auto it = lights.begin(); it != lights.end(); ++it) {
        (*it).transform(transformation);
    }
}

void ndc_to_raster_all(int width, int height, const vector<vec4>& ndc_vertices, vector<vec4> & raster_vertices) {
    //rescale all vertices in ndc_vertices to image plane with given width and height, and store results in raster_vertices
    raster_vertices.resize(ndc_vertices.size());

    auto ntr = std::bind(ndc_to_raster, width, height, _1);

    std::transform(ndc_vertices.begin(),ndc_vertices.end(),raster_vertices.begin(),ntr);
}