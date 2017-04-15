#define TINYOBJLOADER_IMPLEMENTATION
#include <vector>
#include <iostream>
#include <algorithm>
#include <functional>
#include <string>
#include <numeric>
//glm core libraries:
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
//glm extended libraries:
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/extented_min_max.hpp>
#include <glm/gtc/constants.hpp>
//libraries in local vendor folder:
#include "./vendor/cimg/CImg.h"
#include "./vendor/tinyobjloader/tiny_obj_loader.h"
//TCLAP
#include <tclap/CmdLine.h>

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

struct Light {
    vec3 direction;
    float intensity;
    vec3 colour;
    vec3 trans_dir;
    Light(vec3 d, float i, vec3 c) : direction(d), intensity(i), colour(c) { }
    void transform(const mat4& transformation);
};

void add_square(vector<vec3> &vertices, vector<uvec3> &faces) {
    //Load a very simple scene for testing
    vertices.push_back(vec3(-0.5f,-0.5f,0.0f));
    vertices.push_back(vec3(0.5f,-0.5f,0.0f));
    vertices.push_back(vec3(-0.5f,0.5f,0.0f));
    vertices.push_back(vec3(0.5f,0.5f,0.0f));

    faces.push_back(uvec3(0,1,2));
    faces.push_back(uvec3(2,1,3));
}

void load_obj(std::string file, vector<vec3> &vertices, vector<uvec3> &faces) {
    //load a Wavefront .obj file at 'file' and store vertex coordinates as vec3 and faces as uvec3 of indices

    tinyobj::attrib_t attrib;
    vector<tinyobj::shape_t> shapes;
    vector<tinyobj::material_t> materials; //necessary for function call, but will be discarded
    std::string err;

    //load all data in Obj file
    //'triangulate' option defaults to 'true' so all faces should be triangles
    bool success = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, file.c_str());

    //boilerplate error handling
    if (!err.empty()) {
        std::cerr << err << std::endl;
    }
    if (!success) {
        exit(1);
    }

    //convert the vertices into our format
    for(size_t vert = 0; vert < attrib.vertices.size()-2; vert+=3) {
        vertices.push_back(
            vec3(attrib.vertices[vert],
                attrib.vertices[vert+1],
                attrib.vertices[vert+2]
            ));
    }

    //convert the faces into our format
    //faces should all be triangles due to triangulate=true
    for(size_t shape = 0; shape < shapes.size(); shape++) {
        vector<tinyobj::index_t> indices = shapes[shape].mesh.indices;
        for(size_t face = 0; face < indices.size()-2; face+=3) {
            faces.push_back(
                uvec3(indices[face].vertex_index,
                    indices[face+1].vertex_index,
                    indices[face+2].vertex_index
                ));
        }
    }
}

mat4 modelview_matrix(const mat4& model,float angle) {
    //transform model matrix as for a camera pointed at the origin in the xz plane, rotated by angle radians

    mat4 modelview = glm::translate(model, vec3(0.f,0.f,-3.f));
    modelview = glm::rotate(modelview,angle,vec3(0.f,1.f,0.f));

    return modelview;
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

void Light::transform(const mat4& transformation) {
    trans_dir = glm::normalize(transform_direction(transformation,direction));
}

void transform_lights(const mat4& transformation, vector<Light>& lights) {
    for (auto it = lights.begin(); it != lights.end(); ++it) {
        (*it).transform(transformation);
    }
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

float light_contribution(const vec3& normal, float albedo, const Light& light) {
    return light.intensity * albedo * glm::max(0.f,glm::dot(normal,-light.trans_dir)) * glm::one_over_pi<float>();
}

unsigned char shade(const vec3& normal,float albedo, const vector<Light> lights) {
    //determine colour of pixel given barycentric coordinates

    vector<float> light_contributions(lights.size());
    auto lc = std::bind(light_contribution,normal,albedo,_1);
    std::transform(lights.begin(),lights.end(),light_contributions.begin(), lc);
    
    float result = glm::min(std::accumulate(light_contributions.begin(),light_contributions.end(),0.f),255.f);

    return (unsigned char)result;
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
            frame_buffer(raster_x,raster_y,0,0) = shade(normal,1.0,lights); //todo: update shading function
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



int main(int argc,char** argv) {
    try {
        TCLAP::CmdLine cmd("Render a model by rasterisation.", ' ', "0.1");

        TCLAP::ValueArg<float> angleArg("a","angle","Camera view angle",false,0.f,"radians",cmd);
        TCLAP::ValueArg<unsigned int> widthArg("x","width","Width of output in pixels",false,540u,"pixels",cmd);
        TCLAP::ValueArg<unsigned int> heightArg("y","height","Height of output in pixels",false,304u,"pixels",cmd);
        TCLAP::ValueArg<std::string> objArg("o","obj","Wavefront .obj file to load",false,"null","file.obj",cmd);
        
        cmd.parse(argc,argv);

        //define output width and height
        unsigned int image_width = widthArg.getValue();
        unsigned int image_height = heightArg.getValue();
        float aspect_ratio = (float)image_width/(float)image_height;
        float angle = angleArg.getValue();

        //define storage for vertices in various coordinate systems
        vector<vec3> model_vertices;
        vector<vec4> camera_vertices_homo;
        vector<vec3> camera_vertices;
        vector<vec4> clip_vertices;
        vector<vec3> ndc_vertices;
        vector<vec3> raster_vertices;

        //define storage for faces (indices into vertices)
        vector<uvec3> faces;

        //define storage for lights
        vector<Light> lights;

        //add a light from above and in front of scene
        lights.push_back(Light(vec3(0.f,1.f,-1.f),500.f,vec3(1.f)));

        //load model to render
        std::string objFile = objArg.getValue();
        if (objFile!="null") {
            load_obj(objFile,model_vertices,faces);
        } else {
            //load dummy data
            add_square(model_vertices,faces);
        }

        //calculate model-view matrix
        mat4 model(1.0f); //later include per-model model matrix

        mat4 modelview = modelview_matrix(model,angle);

        //add perspective projection to model-view matrix
        mat4 camera = camera_matrix(modelview,aspect_ratio);

        //transform vertices into camera space using model-view matrix for later use in shading
        transform_vertices(modelview, model_vertices, camera_vertices_homo);
        z_divide_all(camera_vertices_homo,camera_vertices);

        transform_lights(modelview,lights);

        //transform vertices into clip space using camera matrix
        transform_vertices(camera, model_vertices, clip_vertices);

        //transform vertices into Normalised Device Coordinates
        z_divide_all(clip_vertices, ndc_vertices);

        //transform Normalised Device Coordinates to raster coordinates given our image
        ndc_to_raster_all(image_width,image_height,ndc_vertices,raster_vertices);

        //instantiate buffers for storing pixel data
        CImg<unsigned char> frame_buffer(image_width,image_height,1,3,0);
        CImg<float> depth_buffer(image_width,image_height,1,1,1.f);

        //for each face in faces, draw it to the frame and depth buffers
        auto dt = std::bind(draw_triangle, _1, raster_vertices, camera_vertices, lights, &frame_buffer, &depth_buffer, image_width, image_height);
        std::for_each(faces.begin(),faces.end(),dt);

        //output frame and depth buffers
        frame_buffer.save("frame.png");
        depth_buffer.normalize(0,255).save("depth.png");
        
        return 0;

    } catch (TCLAP::ArgException &e)  // catch any exceptions
    { std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl; }
}
