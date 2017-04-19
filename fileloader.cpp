#define TINYOBJLOADER_IMPLEMENTATION
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <glm/vec3.hpp>
#include <text/csv/istream.hpp>

#include "tiny_obj_loader.h"

#include "light.h"
#include "fileloader.h"

using glm::vec3;
using glm::uvec3;
using std::vector;

using ::text::csv::csv_istream;

void load_obj(std::string file, vector<vec3> &vertices, vector<uvec3> &faces, vector<vec3> & vertnormals) {
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

    //convert the vertex normals into our format
    for(size_t vert = 0; vert < attrib.normals.size()-2; vert+=3) {
        vertices.push_back(
            vec3(attrib.normals[vert],
                attrib.normals[vert+1],
                attrib.normals[vert+2]
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

void load_lights(std::string file, vector<Light> &lights) {
    std::ifstream file_stream(file);
    csv_istream csv_stream(file_stream);

    float dx, dy, dz, i, r, g, b;

    while(csv_stream) {
        csv_stream >> dx >> dy >> dz >> i >> r >> g >> b;
        lights.push_back(Light(vec3(dx,dy,dz),i,vec3(r,g,b)));
    }
}