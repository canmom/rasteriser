#include <vector>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

#include <glm/gtx/extented_min_max.hpp>

#include "CImg.h"

#include "swizzle.h"
#include "light.h"
#include "shading.h"
#include "geometry.h"
#include "drawing.h"
#include "arguments.h"

using std::vector;

using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::uvec2;
using glm::uvec3;
using glm::mat4;

using cimg_library::CImg;

float edge(const vec2& point, const vec3& vert1, const vec3& vert2) {
    //'edge function' is positive iff point is to the 'left' of the line vert2-vert1
    return (vert2.x-vert1.x)*(point.y-vert1.y) - (vert2.y - vert1.y) * (point.x - vert1.x);
}

vec3 barycentric(const vec2& point, const vec3& vert0, const vec3& vert1, const vec3& vert2) {
    //return 3D vector containing XY plane barycentric coordinates of point with respect to vert0, vert1, and vert2
    //barycentric coordinates are used for linear interpolation of quantities in screen space
    //if all barycentric coordinates are positive, a point lies inside the screen space triangle defined by vert0, vert1 and vert2

    float area = edge(xy(vert2), vert0, vert1);
    vec3 bary(0.0f);

    bary.x = edge(point,vert1,vert2)/area;
    bary.y = edge(point,vert2,vert0)/area;
    bary.z = edge(point,vert0,vert1)/area;

    return bary;
}

template <typename T>
inline T interpolate(T v0, T v1, T v2, const vec3& bary) {
    //linearly interpolate or extrapolate a quantity v defined on three vertices in screen space
    //bary should contain barycentric coordinates with respect to the three vertices
    return bary.x * v0 + bary.y * v1 + bary.z * v2;
}

void bounding_box(uvec2& top_left, uvec2& bottom_right,
    const vec3& vert0, const vec3& vert1, const vec3& vert2,
    unsigned int image_width, unsigned int image_height) {
    //calculate the bounding box of vert0, vert1 and vert2 in the XY plane
    //clamped within image plane [0,0] to [image_width,image_height]
    //store results as top_left and bottom_right corners of bounding box

    vec2 image_bottom_right(image_width - 1, image_height -1);

    top_left = (uvec2)glm::clamp(
        glm::min(xy(vert0),xy(vert1),xy(vert2)),
        vec2(0.f),image_bottom_right);

    bottom_right = (uvec2)glm::clamp(
        glm::ceil(glm::max(xy(vert0),xy(vert1),xy(vert2))),
        vec2(0.f),image_bottom_right);
}


void update_pixel(unsigned int raster_x, unsigned int raster_y,
    const vec3& vert0, const vec3& vert1, const vec3& vert2, const vec3& normal, const vector<Light>& lights,
    CImg<unsigned char>& frame_buffer, CImg<float>& depth_buffer) {
    //take pixel at point raster_x,raster_y in image plane
    //determine if it is inside traingle defined by vert0, vert1 and vert2
    //if so, determine if it is nearer than the current depth buffer
    //if so, update depth buffer and shade pixel

    vec3 bary = barycentric(vec2(raster_x,raster_y),vert0,vert1,vert2);
    float depth = interpolate(vert0.z,vert1.z,vert2.z,bary);

    //Is this pixel inside the triangle?
    if (glm::all(glm::greaterThanEqual(bary,vec3(0.f)))) {
        //Is this pixel nearer than the current value in the depth buffer?
        if(depth < depth_buffer(raster_x,raster_y)) {
            depth_buffer(raster_x,raster_y,0,0) = depth;
            uvec3 pixel = shade(normal,vec3(1.f),lights);
            frame_buffer(raster_x,raster_y,0,0) = (unsigned char)pixel.r;
            frame_buffer(raster_x,raster_y,0,1) = (unsigned char)pixel.g;
            frame_buffer(raster_x,raster_y,0,2) = (unsigned char)pixel.b;
        }
    }
}

void draw_triangle(const uvec3& face, const vector<vec3>& raster_vertices, const vector<vec3>& camera_vertices, const vector<Light>& lights,
    CImg<unsigned char>* frame_buffer, CImg<float>* depth_buffer,
    unsigned int image_width, unsigned int image_height) {
    //draw all the pixels from a triangle to the frame and depth buffers
    //face should contain three indices into raster_vertices

    vec3 vert0_raster = raster_vertices[face.x];
    vec3 vert1_raster = raster_vertices[face.y];
    vec3 vert2_raster = raster_vertices[face.z];

    vec3 vert0_camera = camera_vertices[face.x];
    vec3 vert1_camera = camera_vertices[face.y];
    vec3 vert2_camera = camera_vertices[face.z];

    uvec2 top_left;
    uvec2 bottom_right;

    vec3 normal = glm::normalize(glm::cross(vert1_camera-vert0_camera,vert2_camera-vert0_camera));

    bounding_box(top_left,bottom_right,vert0_raster,vert1_raster,vert2_raster,image_width,image_height);

    //loop over all pixels inside bounding box of triangle
    //call update_pixel on each one to update frame and depth buffers
    for(unsigned int raster_y = top_left.y; raster_y <= bottom_right.y; raster_y++) {
        for(unsigned int raster_x = top_left.x; raster_x <= bottom_right.x; raster_x++) {
            update_pixel(raster_x,raster_y, vert0_raster,vert1_raster,vert2_raster, normal, lights, *frame_buffer,*depth_buffer);
        }
    }
}

void draw_frame(const vector<vec3>& model_vertices, const vector<uvec3>& faces, vector<Light>& lights, const Args& arguments,
    CImg<unsigned char>* frame_buffer, CImg<float>* depth_buffer) {
    //transform the model vertices and draw a frame to the frame buffer

    //define storage for vertices in various coordinate systems
    unsigned int num_vertices = model_vertices.size();
    vector<vec4> camera_vertices_homo(num_vertices);
    vector<vec3> camera_vertices(num_vertices);
    vector<vec4> clip_vertices(num_vertices);
    vector<vec3> ndc_vertices(num_vertices);
    vector<vec3> raster_vertices(num_vertices);

    //calculate model-view matrix
    mat4 model(1.0f); //later include per-model model matrix

    mat4 modelview = modelview_matrix(model,arguments.angle);

    //add perspective projection to model-view matrix
    mat4 camera = camera_matrix(modelview,arguments.aspect_ratio);

    //transform vertices into camera space using model-view matrix for later use in shading
    transform_vertices(modelview, model_vertices, camera_vertices_homo);
    z_divide_all(camera_vertices_homo,camera_vertices);

    transform_lights(modelview,lights);

    //transform vertices into clip space using camera matrix
    transform_vertices(camera, model_vertices, clip_vertices);

    //transform vertices into Normalised Device Coordinates
    z_divide_all(clip_vertices, ndc_vertices);

    //transform Normalised Device Coordinates to raster coordinates given our image
    ndc_to_raster_all(arguments.image_width,arguments.image_height,ndc_vertices,raster_vertices);

    //for each face in faces, draw it to the frame and depth buffers
    for (auto face = faces.begin(); face < faces.end(); ++face) {
        draw_triangle(*face,raster_vertices,camera_vertices,lights,frame_buffer,depth_buffer,arguments.image_width,arguments.image_height);
    }
}