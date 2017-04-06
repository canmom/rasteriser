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
    vertices.push_back(vec3(-0.5f,-0.5f,0.0f));
    vertices.push_back(vec3(0.5f,-0.5f,0.0f));
    vertices.push_back(vec3(-0.5f,0.5f,0.0f));
    vertices.push_back(vec3(0.5f,0.5f,0.0f));

    faces.push_back(uvec3(0,1,2));
    faces.push_back(uvec3(2,1,3));
}

void print_mat4(const mat4& matrix) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            cout << matrix[j][i];
            if (j != 3) {
                cout << ",";
            }
        }
        cout << '\n';
    }
}

void print_vec3(const vec3& vect) {
    cout << vect.x << ',' << vect.y << ',' << vect.z << '\n';
}

void print_vec2(const vec2& vect) {
    cout << vect.x << ',' << vect.y << '\n';
}

void print_uvec3(const uvec3& vect) {
    cout << vect.x << ',' << vect.y << ',' << vect.z << '\n';
}

void print_uvec2(const uvec2& vect) {
    cout << vect.x << ',' << vect.y << '\n';
}

mat4 camera_matrix(float angle) {
    mat4 perspective = glm::perspective(glm::radians(45.0f),16.0f/9.0f,0.1f,3.f);

    mat4 view(1.0f);
    view = glm::translate(view, vec3(0.0f,0.0f,-1.5f));
    view = glm::rotate(view,angle,vec3(0.0f,1.0f,0.0f));

    return perspective * view;
}

vec4 transform_point(const mat4& transformation, const vec3& point) {
    vec4 homo_point(point.x,point.y,point.z,1.0f);
    return transformation * homo_point;
}

void transform_vertices(const mat4& transformation, const vector<vec3>& vertices, vector<vec4>& result) {
    result.resize(vertices.size());

    //partially apply the transform_point function with the given transformation matrix
    auto tp = std::bind(transform_point, transformation, _1);
    
    //apply the transformation to every point and store the results in 'result'
    std::transform(vertices.begin(),vertices.end(),result.begin(),tp);
}

vec3 z_divide(const vec4& clip_vertex) {
    return vec3(
        clip_vertex.x/clip_vertex.w,
        clip_vertex.y/clip_vertex.w,
        clip_vertex.z/clip_vertex.w
    );
}

void z_divide_all(const vector<vec4>& clip_vertices, vector<vec3>& ndc_vertices) {
    ndc_vertices.resize(clip_vertices.size());

    std::transform(clip_vertices.begin(),clip_vertices.end(),ndc_vertices.begin(),z_divide);
}

vec2 xy(const vec3& v) {
    return vec2(v.x,v.y);
}

float edge(const vec2& point, const vec3& vert1, const vec3& vert2) {
    return (vert2.x-vert1.x)*(point.y-vert1.y) - (vert2.y - vert1.y) * (point.x - vert1.x);
}

vec3 barycentric(const vec2& point, const vec3& vert0, const vec3& vert1, const vec3& vert2) {
    float area = edge(xy(vert2), vert0, vert1);
    vec3 bary(0.0f);

    bary.x = edge(point,vert1,vert2)/area;
    bary.y = edge(point,vert2,vert0)/area;
    bary.z = edge(point,vert0,vert1)/area;

    return bary;
}

template <typename T>
inline T interpolate(T v0, T v1, T v2, const vec3& bary) {
    return bary.x * v0 + bary.y * v1 + bary.z * v2;
}

float remap_ndc(float value, float high) {
    return 0.5f*(value + 1.0f)*high;
}

vec3 ndc_to_raster(int width,int height,const vec3& ndc_vertex) {
    return vec3(
        remap_ndc(ndc_vertex.x,width),
        remap_ndc(-ndc_vertex.y,height),
        ndc_vertex.z);
}

void ndc_to_raster_all(int width, int height, const vector<vec3>& ndc_vertices, vector<vec3> & raster_vertices) {
    raster_vertices.resize(ndc_vertices.size());

    auto ntr = std::bind(ndc_to_raster, width, height, _1);

    std::transform(ndc_vertices.begin(),ndc_vertices.end(),raster_vertices.begin(),ntr);
}

// template <typename T>
// inline T clamp (T value,T low,T high) {
//     return value < low ? low : (value > high ? high : value);
// }

void bounding_box(uvec2& top_left, uvec2& bottom_right,
    const vec3& vert0, const vec3& vert1, const vec3& vert2,
    unsigned int image_width, unsigned int image_height) {

    // top_left.x = (unsigned int)glm::clamp(std::min({vert0.x,vert1.x,vert2.x}),0.f,image_width - 1.f);
    // top_left.y = (unsigned int)glm::clamp(std::min({vert0.y,vert1.y,vert2.y}),0.f,image_height - 1.f);

    // bottom_right.x = (unsigned int)glm::clamp(std::max({vert0.x,vert1.x,vert2.x}),0.f,image_width - 1.f);
    // bottom_right.y = (unsigned int)glm::clamp(std::max({vert0.y,vert1.y,vert2.y}),0.f,image_height - 1.f);

    vec2 image_bottom_right(image_width - 1, image_height -1);

    top_left = (uvec2)glm::clamp(
        glm::min(xy(vert0),xy(vert1),xy(vert2)),
        vec2(0.f),image_bottom_right);

    bottom_right = (uvec2)glm::clamp(
        glm::ceil(glm::max(xy(vert0),xy(vert1),xy(vert2))),
        vec2(0.f),image_bottom_right);
}

unsigned char shade(const vec3& bary) {
    return 255;
}

void update_pixel(unsigned int raster_x, unsigned int raster_y,
    const vec3& vert0, const vec3& vert1, const vec3& vert2,
    CImg<unsigned char>& frame_buffer, CImg<float>& depth_buffer) {

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

    vec3 vert0 = raster_vertices[face.x];
    vec3 vert1 = raster_vertices[face.y];
    vec3 vert2 = raster_vertices[face.z];

    uvec2 top_left;
    uvec2 bottom_right;

    bounding_box(top_left,bottom_right,vert0,vert1,vert2,image_width,image_height);

    cout << "Bounding box: (" << top_left.x << ", " << top_left.y << ") to (" << bottom_right.x << ", " << bottom_right.y << ")\n";

    for(unsigned int raster_y = top_left.y; raster_y <= bottom_right.y; raster_y++) {
        for(unsigned int raster_x = top_left.x; raster_x <= bottom_right.x; raster_x++) {
            update_pixel(raster_x,raster_y, vert0,vert1,vert2, *frame_buffer,*depth_buffer);

        }
    }
}

int main() {
    unsigned int image_width = 540;
    unsigned int image_height = 304;

    vector<vec3> vertices;
    vector<vec4> clip_vertices;
    vector<vec3> ndc_vertices;
    vector<vec3> raster_vertices;
    vector<uvec3> faces;

    add_square(vertices,faces);

    mat4 camera = camera_matrix(1.0f);

    transform_vertices(camera, vertices, clip_vertices);

    z_divide_all(clip_vertices, ndc_vertices);

    ndc_to_raster_all(image_width,image_height,ndc_vertices,raster_vertices);

    CImg<unsigned char> frame_buffer(image_width,image_height,1,3,0);
    CImg<float> depth_buffer(image_width,image_height,1,1,1.f);

    auto dt = std::bind(draw_triangle, _1, raster_vertices, &frame_buffer, &depth_buffer, image_width, image_height);
    std::for_each(faces.begin(),faces.end(),dt);

    //draw_triangle(faces[0], raster_vertices, frame_buffer,depth_buffer,image_width,image_height);

    frame_buffer.save("frame.png");
    depth_buffer.normalize(0,255).save("depth.png");
    
    return 0;
}
