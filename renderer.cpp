#include <vector>
#include <iostream>
#include <algorithm>
#include <functional>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/extented_min_max.hpp>

#include "./vendor/cimg/CImg.h"

using std::vector;
using std::cout;
using namespace std::placeholders;

using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::mat4;
using glm::uvec2;
using glm::uvec3;

using cimg_library::CImg;

void add_square(vector<vec3> &vertices, vector<uvec3> &faces) {
    //Load a very simple scene for testing
    vertices.push_back(vec3(-0.5f,-0.5f,0.0f));
    vertices.push_back(vec3(0.5f,-0.5f,0.0f));
    vertices.push_back(vec3(-0.5f,0.5f,0.0f));
    vertices.push_back(vec3(0.5f,0.5f,0.0f));

    faces.push_back(uvec3(0,1,2));
    faces.push_back(uvec3(2,1,3));
}

mat4 camera_matrix(float angle) {
    //compute the model-view-projection matrix for a camera rotated about the origin by angle radians
    mat4 perspective = glm::perspective(glm::radians(45.0f),16.0f/9.0f,0.1f,3.f);

    mat4 view(1.0f);
    view = glm::translate(view, vec3(0.0f,0.0f,-1.5f));
    view = glm::rotate(view,angle,vec3(0.0f,1.0f,0.0f));

    return perspective * view;
}

vec4 transform_point(const mat4& transformation, const vec3& point) {
    //transform a 3D vector (implicitly in homogeneous coordinates with w=1) using a 4x4 matrix
    //and return as a 4-vector in homogeneous coordinates

    vec4 homo_point(point.x,point.y,point.z,1.0f);
    return transformation * homo_point;
}

void transform_vertices(const mat4& transformation, const vector<vec3>& vertices, vector<vec4>& result) {
    //Apply transformation to each vec3 in vertices and store results in result

    result.resize(vertices.size());

    //partially apply the transform_point function with the given transformation matrix
    auto tp = std::bind(transform_point, transformation, _1);
    
    //apply the transformation to every point and store the results in 'result'
    std::transform(vertices.begin(),vertices.end(),result.begin(),tp);
}

vec3 z_divide(const vec4& clip_vertex) {
    //take 4D vector clip_vertex in homogeneous coordinates and return equivalent 3D vector
    return vec3(
        clip_vertex.x/clip_vertex.w,
        clip_vertex.y/clip_vertex.w,
        clip_vertex.z/clip_vertex.w
    );
}

void z_divide_all(const vector<vec4>& clip_vertices, vector<vec3>& ndc_vertices) {
    //apply z_devide to all vertices in clip_vertices, and store result in ndc_vertices

    ndc_vertices.resize(clip_vertices.size());

    std::transform(clip_vertices.begin(),clip_vertices.end(),ndc_vertices.begin(),z_divide);
}

vec2 xy(const vec3& v) {
    //return 2D vector containing the first two components of v

    return vec2(v.x,v.y);
}

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

float remap_ndc(float value, float high) {
    //scale a quantity defined in the range [-1,1] to the range [0,high]
    return 0.5f*(value + 1.0f)*high;
}

vec3 ndc_to_raster(int width,int height,const vec3& ndc_vertex) {
    //return vertex with X and Y coordinates scaled from Normalised Device Coordinates to an image plane with given width and height
    return vec3(
        remap_ndc(ndc_vertex.x,width),
        remap_ndc(-ndc_vertex.y,height),
        ndc_vertex.z);
}

void ndc_to_raster_all(int width, int height, const vector<vec3>& ndc_vertices, vector<vec3> & raster_vertices) {
    //rescale all vertices in ndc_vertices to image plane with given width and height, and store results in raster_vertices
    raster_vertices.resize(ndc_vertices.size());

    auto ntr = std::bind(ndc_to_raster, width, height, _1);

    std::transform(ndc_vertices.begin(),ndc_vertices.end(),raster_vertices.begin(),ntr);
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

unsigned char shade(const vec3& bary) {
    //determine colour of pixel given barycentric coordinates
    //NOT IMPLEMENTED
    return 255;
}

void update_pixel(unsigned int raster_x, unsigned int raster_y,
    const vec3& vert0, const vec3& vert1, const vec3& vert2,
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
            frame_buffer(raster_x,raster_y,0,0) = shade(bary/* some other vertex data*/); //todo: update shading function
        }
    }
}

void draw_triangle(const uvec3& face, const vector<vec3>& raster_vertices,
    CImg<unsigned char>* frame_buffer, CImg<float>* depth_buffer,
    unsigned int image_width, unsigned int image_height) {
    //draw all the pixels from a triangle to the frame and depth buffers
    //take three indices into raster_vertices in face
    //and pointers to the frame and depth buffers
    //loop over all pixels inside bounding box of triangle
    //call update_pixel on each one to update frame and depth buffers

    vec3 vert0 = raster_vertices[face.x];
    vec3 vert1 = raster_vertices[face.y];
    vec3 vert2 = raster_vertices[face.z];

    uvec2 top_left;
    uvec2 bottom_right;

    bounding_box(top_left,bottom_right,vert0,vert1,vert2,image_width,image_height);

    for(unsigned int raster_y = top_left.y; raster_y <= bottom_right.y; raster_y++) {
        for(unsigned int raster_x = top_left.x; raster_x <= bottom_right.x; raster_x++) {
            update_pixel(raster_x,raster_y, vert0,vert1,vert2, *frame_buffer,*depth_buffer);

        }
    }
}

int main() {
    //define output width and height
    unsigned int image_width = 540;
    unsigned int image_height = 304;

    //define storage for vertices in various coordinate systems
    vector<vec3> vertices;
    vector<vec4> clip_vertices;
    vector<vec3> ndc_vertices;
    vector<vec3> raster_vertices;

    //define storage for faces (indices into vertices)
    vector<uvec3> faces;

    //load dummy data
    add_square(vertices,faces);

    //calculate camera matrix for a camera rotated by 1 radian
    mat4 camera = camera_matrix(1.0f);

    //transform vertices into clip space using camera matrix
    transform_vertices(camera, vertices, clip_vertices);

    //transform vertices into Normalised Device Coordinates
    z_divide_all(clip_vertices, ndc_vertices);

    //transform Normalised Device Coordinates to raster coordinates given our image
    ndc_to_raster_all(image_width,image_height,ndc_vertices,raster_vertices);

    //instantiate buffers for storing pixel data
    CImg<unsigned char> frame_buffer(image_width,image_height,1,3,0);
    CImg<float> depth_buffer(image_width,image_height,1,1,1.f);

    //for each face in faces, draw it to the frame and depth buffers
    auto dt = std::bind(draw_triangle, _1, raster_vertices, &frame_buffer, &depth_buffer, image_width, image_height);
    std::for_each(faces.begin(),faces.end(),dt);

    //output frame and depth buffers
    frame_buffer.save("frame.png");
    depth_buffer.normalize(0,255).save("depth.png");
    
    return 0;
}
