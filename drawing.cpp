#include <vector>
#include <array>

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
#include "face.h"

using std::vector;
using std::array;

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

template <typename T>
inline T perspective_interpolate(T v0, T v1, T v2, float depth, const vec3& vert_depth, float offset, const vec3& bary) {
    //perspective-correct linearly interpolate or extrapolate a quantity v defined on three vertices in screen space
    //depth is the NDC depth of the target point
    //vert_depth should contain the NDC depths of the three vertices
    //offset should contain a correction equal to M_33 where M is the perspective projection matrix
    //bary should contain barycentric coordinates with respect to the three vertices
    return depth * (bary.x*vert_depth.x*v0 + bary.y*vert_depth.y*v1 + bary.z*vert_depth.z*v2 - offset * interpolate(v0,v1,v2,bary));
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
    const array<vec3,3>& raster_vertices,
    vec3& normal, const array<vec3,3>& vertnormals, const array<vec2,3> vertuvs, const vector<Light>& lights,
    CImg<unsigned char>& frame_buffer, CImg<float>& depth_buffer, bool flat, bool wind_clockwise, float z_offset) {
    //take pixel at point raster_x,raster_y in image plane
    //determine if it is inside traingle defined by vert0, vert1 and vert2 (each in raster coordinates)
    //if so, determine if it is nearer than the current depth buffer
    //if so, update depth buffer and shade pixel

    vec3 bary = barycentric(vec2(raster_x,raster_y),raster_vertices[0],raster_vertices[1],raster_vertices[2]);

    //determine the Normalised Device Coordinate depth value
    float depth = interpolate(raster_vertices[0].z,raster_vertices[1].z,raster_vertices[2].z,bary);

    //if smooth shading is enabled, update normal according to position in triangle
    if(not flat) {
        normal = glm::normalize(perspective_interpolate(vertnormals[0],vertnormals[1],vertnormals[2],
            depth,vec3(raster_vertices[0].z,raster_vertices[1].z,raster_vertices[2].z),
            z_offset,bary));
        if (wind_clockwise) {normal = -normal;}
    }

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

void draw_triangle(const Triangle& face, const vector<vec3>& raster_vertices,
    const vector<vec3>& camera_vertices, const vector<vec3>& camera_vertnormals, const vector<vec2>& vertuvs, const vector<Light>& lights,
    CImg<unsigned char>* frame_buffer, CImg<float>* depth_buffer,
    unsigned int image_width, unsigned int image_height, bool flat, bool wind_clockwise, float z_offset) {
    //draw all the pixels from a triangle to the frame and depth buffers
    //face should contain three indices into raster_vertices

    array<vec3,3> face_raster_vertices;
    array<vec3,3> face_camera_vertices;
    array<vec3,3> face_camera_vertnormals;
    array<vec2,3> face_vertuvs;
    for (size_t facevert = 0; facevert < 3; facevert++) {
        face_raster_vertices[facevert] = raster_vertices[face.vertices[facevert]];
        face_camera_vertices[facevert] = camera_vertices[face.vertices[facevert]];
        face_vertuvs[facevert] = vertuvs[face.uvs[facevert]];
        if (not flat) {face_camera_vertnormals[facevert] = camera_vertnormals[face.normals[facevert]];}
    }

    uvec2 top_left;
    uvec2 bottom_right;
    vec3 face_normal = glm::normalize(glm::cross(face_camera_vertices[1]-face_camera_vertices[0],face_camera_vertices[2]-face_camera_vertices[0]));
    if (wind_clockwise) {face_normal = -face_normal;};

    if (face_normal.z > 0.f) { //backface culling
        bounding_box(top_left,bottom_right,
            face_raster_vertices[0],face_raster_vertices[1],face_raster_vertices[2],image_width,image_height);

        //loop over all pixels inside bounding box of triangle
        //call update_pixel on each one to update frame and depth buffers
        for(unsigned int raster_y = top_left.y; raster_y <= bottom_right.y; raster_y++) {
            for(unsigned int raster_x = top_left.x; raster_x <= bottom_right.x; raster_x++) {
                update_pixel(raster_x,raster_y,
                    face_raster_vertices,
                    face_normal, face_camera_vertnormals, face_vertuvs, lights,
                    *frame_buffer,*depth_buffer,
                    flat, wind_clockwise,
                    z_offset);
            }
        }
    }
}

void draw_frame(const vector<vec3>& model_vertices, const vector<Triangle>& faces,
    const vector<vec3>&model_vertnormals, const vector<vec2>&vertuvs, vector<Light>& lights, const Args& arguments,
    CImg<unsigned char>* frame_buffer, CImg<float>* depth_buffer) {
    //transform the model vertices and draw a frame to the frame buffer

    //define storage for vertices in various coordinate systems
    unsigned int num_vertices = model_vertices.size();
    vector<vec4> camera_vertices_homo(num_vertices);
    vector<vec3> camera_vertices(num_vertices);
    vector<vec4> clip_vertices(num_vertices);
    vector<vec3> ndc_vertices(num_vertices);
    vector<vec3> raster_vertices(num_vertices);

    //storage for vertex normals in camera space
    vector<vec3> camera_vertnormals(num_vertices);

    //quantity required for perspective-correct interpolation
    float z_offset;

    //calculate model-view matrix
    mat4 model = transformation_matrix(arguments.scale,arguments.displacement,arguments.tait_bryan_angles);

    mat4 view = transformation_matrix(1.f,vec3(0.f,0.f,-3.f),vec3(0.f));

    mat4 modelview = view * model;

    //add perspective projection to model-view matrix
    mat4 camera = camera_matrix(modelview,arguments.aspect_ratio,z_offset);

    //transform vertices into camera space using model-view matrix for later use in shading
    transform_vertices(modelview, model_vertices, camera_vertices_homo);
    xyz_all(camera_vertices_homo,camera_vertices);

    //transform normals into camera space
    transform_normals(modelview,model_vertnormals,camera_vertnormals);

    transform_lights(view,lights);

    //transform vertices into clip space using camera matrix
    transform_vertices(camera, model_vertices, clip_vertices);

    //transform vertices into Normalised Device Coordinates
    z_divide_all(clip_vertices, ndc_vertices);

    //transform Normalised Device Coordinates to raster coordinates given our image
    ndc_to_raster_all(arguments.image_width,arguments.image_height,ndc_vertices,raster_vertices);

    //for each face in faces, draw it to the frame and depth buffers
    for (auto face = faces.begin(); face < faces.end(); ++face) {
        draw_triangle(*face,raster_vertices,
            camera_vertices,camera_vertnormals,vertuvs,lights,
            frame_buffer,depth_buffer,
            arguments.image_width, arguments.image_height,
            arguments.flat, arguments.wind_clockwise,
            z_offset);
    }
}