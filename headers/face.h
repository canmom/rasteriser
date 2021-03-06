#ifndef RASTERISER_FACE_H
#define RASTERISER_FACE_H

#include <array>

struct Triangle {
	//indices into vectors of coordinates, vertex normals, and uv texture coordinates for each vertex
	std::array<int,3> vertices;
	std::array<int,3> normals;
	std::array<int,3> uvs;
	int material;
	Triangle(std::array<int,3> v, std::array<int,3> n, std::array<int,3> uv, int m) : vertices(v), normals(n), uvs(uv), material(m) {};
};

#endif