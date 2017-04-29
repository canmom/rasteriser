#define TINYOBJLOADER_IMPLEMENTATION
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <array>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <text/csv/istream.hpp>

#include "tiny_obj_loader.h"
#include "CImg.h"

#include "light.h"
#include "fileloader.h"
#include "face.h"
#include "material.h"

using cimg_library::CImg;

using glm::vec2;
using glm::vec3;
using glm::uvec3;
using std::vector;
using std::array;

using ::text::csv::csv_istream;

void components_to_vec2s(const vector<float> components, vector<vec2>& vecs) {
    for(size_t vec_start = 0; vec_start < components.size(); vec_start+=2) {
        vecs.push_back(
            vec2(components[vec_start],
                components[vec_start+1]
            ));
    }
}

void components_to_vec3s(const vector<float> components, vector<vec3>& vecs) {
    //convert a vector of back-to-back vertex components to a vector of vec3 objects
    for(size_t vec_start = 0; vec_start < components.size(); vec_start+=3) {
        vecs.push_back(
            vec3(components[vec_start],
                components[vec_start+1],
                components[vec_start+2]
            ));
    }
}

void load_materials(const vector<tinyobj::material_t> & objmaterials, vector<Material> & materials) {
    //extract the relevant material properties from the material format used by tinyobjloader
    for(auto mat = objmaterials.begin(); mat < objmaterials.end(); ++mat) {
        vec3 diffuse_colour((*mat).diffuse[0],(*mat).diffuse[1],(*mat).diffuse[2]);
        if ((*mat).diffuse_texname.empty()) {
            materials.push_back(Material(diffuse_colour));
        } else {
            materials.push_back(Material(diffuse_colour,(*mat).diffuse_texname));
        }
    }
}

void load_triangles(const tinyobj::shape_t & shape, vector<Triangle> & triangles) {
    const vector<tinyobj::index_t> & indices = shape.mesh.indices;
    const vector<int> & mat_ids = shape.mesh.material_ids;

    std::cout << "Loading " << mat_ids.size() << " triangles..." << std::endl;

    for(size_t face = 0; face < mat_ids.size(); face++) {
        triangles.push_back(
            Triangle(
                {indices[3*face].vertex_index, indices[3*face+1].vertex_index, indices[3*face+2].vertex_index},
                {indices[3*face].normal_index, indices[3*face+1].normal_index, indices[3*face+2].normal_index},
                {indices[3*face].texcoord_index, indices[3*face+1].texcoord_index, indices[3*face+2].texcoord_index},
                mat_ids[face]
                ));
    }
}

void load_obj(std::string file, vector<vec3> &vertices, vector<Triangle> &triangles, vector<vec3> & vertnormals, vector<vec2>& vertuvs, vector<Material>& materials) {
    //load a Wavefront .obj file at 'file' and store vertex coordinates as vec3 and faces as uvec3 of indices

    tinyobj::attrib_t attrib;
    vector<tinyobj::shape_t> shapes;
    vector<tinyobj::material_t> objmaterials;
    std::string err;

    //load all data in Obj file
    //'triangulate' option defaults to 'true' so all faces should be triangles
    bool success = tinyobj::LoadObj(&attrib, &shapes, &objmaterials, &err, file.c_str());

    //boilerplate error handling
    if (!err.empty()) {
        std::cerr << err << std::endl;
    }
    if (!success) {
        exit(1);
    }

    //convert the vertices into our format
    components_to_vec3s(attrib.vertices, vertices);

    //convert the vertex normals into our format
    components_to_vec3s(attrib.normals, vertnormals);

    //convert the uv coordinates into our format
    components_to_vec2s(attrib.texcoords, vertuvs);

    //conver materials and load textures
    load_materials(objmaterials, materials);

    //convert the faces into our format
    //faces should all be triangles due to triangulate=true
    for(auto shape = shapes.begin(); shape < shapes.end(); ++shape) {
        load_triangles(*shape, triangles);
    }

    std::cout << "Loaded model " << file << "." << std::endl;
}

void load_lights(std::string file, vector<Light> &lights) {
    std::ifstream file_stream(file);
    csv_istream csv_stream(file_stream);

    float dx, dy, dz, i, r, g, b;

    while(csv_stream) {
        csv_stream >> dx >> dy >> dz >> i >> r >> g >> b;
        lights.push_back(Light(vec3(dx,dy,dz),i,vec3(r,g,b)));
    }
}