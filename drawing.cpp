#include <vector>
#include <array>
#include <algorithm>
#include <functional>
#include <iostream>

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
#include "material.h"

using std::vector;
using std::array;
using std::transform;

using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::uvec2;
using glm::uvec3;
using glm::mat4;

using cimg_library::CImg;

inline float edge(const vec2& point, const vec4& vert1, const vec4& vert2) {
    //'edge function' is positive iff point is to the 'left' of the line vert2-vert1
    return (vert2.x-vert1.x)*(point.y-vert1.y) - (vert2.y - vert1.y) * (point.x - vert1.x);
}

vec3 barycentric(const vec2& point, const array<vec4,3> & verts) {
    //return 3D vector containing XY plane barycentric coordinates of point with respect to vert0, vert1, and vert2
    //barycentric coordinates are used for linear interpolation of quantities in screen space
    //if all barycentric coordinates are positive, a point lies inside the screen space triangle defined by vert0, vert1 and vert2

    float area = edge(xy(verts[2]), verts[0], verts[1]);

    return vec3(edge(point,verts[1],verts[2])/area, edge(point,verts[2],verts[0])/area, edge(point,verts[0],verts[1])/area);
}

template <typename T>
inline T screen_interpolate(const array<T,3> & vert_values, const vec3& bary) {
    //linearly interpolate or extrapolate a quantity v defined on three vertices in screen space
    //bary should contain barycentric coordinates with respect to the three vertices
    return vert_values[0] * bary[0] + vert_values[1] * bary[1] + vert_values[2] * bary[2];
}

inline vec3 interpolation_coords(const vec3 & inverse_depths, const vec3& bary) {
    //calculate the elementwise products of inverse depths and barycentric coordinates and store it in inter_coords

    return inverse_depths * bary;
}

template <typename T>
inline T perspective_interpolate(const array<T,3> & vert_values, float depth, const vec3 & interpolation_coords) {
    //perspective-correct linearly interpolate or extrapolate a quantity v defined on three vertices in screen space
    //the interpolation coordinates are the products of inverse camera space depths and barycentric coordinates
    //the depth should be the inverse of the sum of the interpolation coordinates
    //bary should contain barycentric coordinates with respect to the three vertices
    
    return depth * (
        interpolation_coords[0] * vert_values[0] +
        interpolation_coords[1] * vert_values[1] +
        interpolation_coords[2] * vert_values[2]);
}

void bounding_box(uvec2& top_left, uvec2& bottom_right,
    const array<vec4,3>& verts,
    unsigned int image_width, unsigned int image_height) {
    //calculate the bounding box of vert0, vert1 and vert2 in the XY plane
    //clamped within image plane [0,0] to [image_width,image_height]
    //store results as top_left and bottom_right corners of bounding box

    vec2 image_bottom_right(image_width - 1, image_height -1);

    top_left = (uvec2)glm::clamp(
        glm::min(xy(verts[0]),xy(verts[1]),xy(verts[2])),
        vec2(0.f),image_bottom_right);

    bottom_right = (uvec2)glm::clamp(
        glm::ceil(glm::max(xy(verts[0]),xy(verts[1]),xy(verts[2]))),
        vec2(0.f),image_bottom_right);
}


void update_pixel(unsigned int raster_x, unsigned int raster_y,
    const array<vec4,3>& raster_vertices,
    const array<vec3,3>& vertnormals, const array<vec2,3>& vertuvs,
    const vector<Light>& lights, const Material& material,
    CImg<unsigned char>& frame_buffer, CImg<float>& depth_buffer,
    bool wind_clockwise) {
    //take pixel at point raster_x,raster_y in image plane
    //determine if it is inside triangle defined by raster_vertices
    //if so, determine if it is nearer than the current depth buffer
    //if so, update depth buffer and shade pixel

    //determine the barycentric coordinates of this point
    vec3 bary = barycentric(vec2(raster_x,raster_y),raster_vertices);

    //Is this pixel inside the triangle?
    if (glm::all(glm::greaterThanEqual(bary,vec3(0.f)))) {

        //depth buffer algorithm:
        //determine the Normalised Device Coordinate depth value
        array<float,3> ndcdepths = {raster_vertices[0].z,raster_vertices[1].z,raster_vertices[2].z};
        float ndcdepth = screen_interpolate(ndcdepths,bary);

        //Is this pixel nearer than the current value in the depth buffer?
        if(ndcdepth < depth_buffer(raster_x,raster_y)) {
            //update the depth buffer
            depth_buffer(raster_x,raster_y,0,0) = ndcdepth;

            //interpolation of values on vertices:
            //determine the perspective-correct interpolation coordinates (barycentric divided coordinates by camera space depth of that pixel)
            vec3 inter = interpolation_coords(vec3(raster_vertices[0].w,raster_vertices[1].w,raster_vertices[2].w),bary);

            //determine the camera-space depth of the point:
            float depth = 1.f/(inter[0]+inter[1]+inter[2]);

            //interpolate vertex normals
            vec3 normal = glm::normalize(perspective_interpolate(vertnormals, depth, inter));
            if (wind_clockwise) {normal = -normal;}

            //interpolate uv coordinates
            vec2 uv = perspective_interpolate(vertuvs, depth, inter);

            //work out what colour this pixel should be (in OpenGL terms, run the fragment shader)
            uvec3 pixel = shade(
                normal,
                material.sample(uv), //if there is a texture, use the texcoord-appropriate colour
                lights);

            //update the frame buffer with the new colour
            frame_buffer(raster_x,raster_y,0,0) = (unsigned char)pixel.r;
            frame_buffer(raster_x,raster_y,0,1) = (unsigned char)pixel.g;
            frame_buffer(raster_x,raster_y,0,2) = (unsigned char)pixel.b;
        }
    }
}

void draw_triangle(const Triangle& face, const vector<vec4>& raster_vertices,
    const vector<vec3>& camera_vertnormals, const vector<vec2>& vertuvs,
    const vector<Light>& lights, const vector<Material>& materials,
    CImg<unsigned char>* frame_buffer, CImg<float>* depth_buffer,
    unsigned int image_width, unsigned int image_height,
    bool wind_clockwise) {
    //draw all the pixels from a triangle to the frame and depth buffers
    //face should contain three indices into raster_vertices

    array<vec4,3> face_raster_vertices;
    array<vec3,3> face_camera_vertnormals;
    array<vec2,3> face_vertuvs;

    //de-index the vertex data associated with this face
    for (size_t facevert = 0; facevert < 3; facevert++) {
        face_raster_vertices[facevert] = raster_vertices[face.vertices[facevert]];
        //check if uv coordinates exist
        if (face.uvs[facevert] >= 0) {
            face_vertuvs[facevert] = vertuvs[face.uvs[facevert]];
        }
        face_camera_vertnormals[facevert] = camera_vertnormals[face.normals[facevert]];
    }
    const Material& face_material = materials[face.material];

    //use signed area to determine whether the face is a front or back face
    float face_raster_signedarea = signed_area_2d(face_raster_vertices);
    //if wind_clockwise is set, a negative value is a front face, otherwise a positive value is
    bool front_face = (face_raster_signedarea > 0) xor wind_clockwise;

    if (front_face) { //backface culling

        //determine the corners of the bounding box for this face
        //define storage for the bounding box corners
        uvec2 top_left;
        uvec2 bottom_right;

        bounding_box(top_left,bottom_right,
            face_raster_vertices,image_width,image_height);

        //loop over all pixels inside bounding box of triangle
        for(unsigned int raster_y = top_left.y; raster_y <= bottom_right.y; raster_y++) {
            for(unsigned int raster_x = top_left.x; raster_x <= bottom_right.x; raster_x++) {
                //call update_pixel on each one to shade and update frame and depth buffers if appropriate
                update_pixel(raster_x,raster_y,
                    face_raster_vertices,
                    face_camera_vertnormals, face_vertuvs,
                    lights, face_material,
                    *frame_buffer,*depth_buffer,
                    wind_clockwise);
            }
        }
    }
}

void draw_frame(const vector<vec3>& model_vertices, const vector<Triangle>& faces,
    const vector<vec3>&model_vertnormals, const vector<vec2>&vertuvs, vector<Light>& lights, const vector<Material>& materials, const Args& arguments,
    CImg<unsigned char>* frame_buffer, CImg<float>* depth_buffer) {
    //transform the model vertices and draw a frame to the frame buffer

    //define storage for vertices in various coordinate systems
    unsigned int num_vertices = model_vertices.size();
    vector<vec4> camera_vertices_homo(num_vertices);
    vector<vec3> camera_vertices(num_vertices);
    vector<vec4> clip_vertices(num_vertices);
    vector<vec4> ndc_vertices(num_vertices);
    vector<vec4> raster_vertices(num_vertices);

    //storage for vertex normals in camera space
    vector<vec3> camera_vertnormals(num_vertices);

    //calculate model-view matrix
    mat4 model = transformation_matrix(arguments.scale,arguments.displacement,arguments.tait_bryan_angles);

    mat4 view = transformation_matrix(1.f,vec3(0.f,0.f,-3.f),vec3(0.f));

    mat4 modelview = view * model;

    //add perspective projection to model-view matrix
    mat4 camera = camera_matrix(modelview,arguments.aspect_ratio);

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
            camera_vertnormals,vertuvs,
            lights,materials,
            frame_buffer,depth_buffer,
            arguments.image_width, arguments.image_height,
            arguments.wind_clockwise);
    }
}