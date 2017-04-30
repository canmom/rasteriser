#define TINYOBJLOADER_IMPLEMENTATION
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <text/csv/istream.hpp>

#include "tiny_obj_loader.h"

#include "light.h"
#include "fileloader.h"
#include "face.h"
#include "material.h"
#include "arguments.h"

using glm::vec2;
using glm::vec3;
using glm::uvec3;
using std::vector;

using ::text::csv::csv_istream;

void components_to_vec2s(const vector<float> components, vector<vec2>& vecs) {
    //convert a vector of back-to-back vertex components to a vector of vec2 objects
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

void load_materials(const vector<tinyobj::material_t> & objmaterials, const std::string & tex_dir, vector<Material> & materials) {
    //extract the relevant material properties from the material_t format used by tinyobjloader
    //if a texture is defined, use the constructor that loads a texture
    for(auto mat = objmaterials.begin(); mat < objmaterials.end(); ++mat) {
        vec3 diffuse_colour((*mat).diffuse[0],(*mat).diffuse[1],(*mat).diffuse[2]);
        if ((*mat).diffuse_texname.empty()) {
            materials.push_back(Material(diffuse_colour));
        } else {
            materials.push_back(Material(diffuse_colour,tex_dir+(*mat).diffuse_texname));
        }
    }
}

void load_triangles(const tinyobj::shape_t & shape, vector<Triangle> & triangles) {
    //convert a tinyobjloader shape_t object containing indices into vertex properties and textures
    //into a vector of Triangle objects grouping these indices
    const vector<tinyobj::index_t> & indices = shape.mesh.indices;
    const vector<int> & mat_ids = shape.mesh.material_ids;

    std::cout << "Loading " << mat_ids.size() << " triangles..." << std::endl;

    for(size_t face_ind = 0; face_ind < mat_ids.size(); face_ind++) {
        triangles.push_back(
            Triangle(
                {indices[3*face_ind].vertex_index, indices[3*face_ind+1].vertex_index, indices[3*face_ind+2].vertex_index},
                {indices[3*face_ind].normal_index, indices[3*face_ind+1].normal_index, indices[3*face_ind+2].normal_index},
                {indices[3*face_ind].texcoord_index, indices[3*face_ind+1].texcoord_index, indices[3*face_ind+2].texcoord_index},
                mat_ids[face_ind]
                ));
    }
}

void load_obj(const Args & args, vector<vec3> &vertices, vector<Triangle> &triangles, vector<vec3> & vertnormals, vector<vec2>& vertuvs, vector<Material>& materials) {
    //load a Wavefront .obj file at 'file' and store vertex coordinates as vec3 and face_inds as uvec3 of indices

    tinyobj::attrib_t attrib;
    vector<tinyobj::shape_t> shapes;
    vector<tinyobj::material_t> objmaterials;
    std::string err;

    //load all data in Obj file
    //'triangulate'
    bool success = tinyobj::LoadObj(&attrib, &shapes, &objmaterials, &err,
        args.obj_file.c_str(), //model to load
        args.materials_directory.c_str(), //directory to search for materials
        true); //enable triangulation

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
    load_materials(objmaterials, args.materials_directory, materials);

    //convert the face_inds into our format
    //face_inds should all be triangles due to triangulate=true
    for(auto shape = shapes.begin(); shape < shapes.end(); ++shape) {
        load_triangles(*shape, triangles);
    }

    std::cout << "Loaded model " << args.obj_file << "." << std::endl;
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