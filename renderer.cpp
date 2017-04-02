#include <vector>
#include <iostream>
#include <algorithm>
#include <functional>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>

#include <glm/gtc/matrix_transform.hpp>

using std::vector;
using std::cout;
using namespace std::placeholders;

using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::mat4;

void add_square(vector<vec3> &vertices, vector<glm::uvec3> &faces) {
    vertices.push_back(vec3(-0.5f,-0.5f,0.0f));
    vertices.push_back(vec3(0.5f,-0.5f,0.0f));
    vertices.push_back(vec3(-0.5f,0.5f,0.0f));
    vertices.push_back(vec3(0.5f,0.5f,0.0f));

    faces.push_back(glm::uvec3(1,0,2));
    faces.push_back(glm::uvec3(1,2,3));
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

vec2 print_display_coords(const vec3& ndc_vertex, const float& display_width, const float& display_height) {
    cout << 0.5f*(ndc_vertex.x+1.0f) * display_width << ',' << 0.5f*(ndc_vertex.y+1.0f) * display_height << '\n'; 
}

int main() {
    vector<vec3> vertices;
    vector<vec4> clip_vertices;
    vector<vec3> ndc_vertices;
    vector<glm::uvec3> faces;

    add_square(vertices,faces);

    mat4 camera = camera_matrix(1.0f);

    transform_vertices(camera, vertices, clip_vertices);

    z_divide_all(clip_vertices, ndc_vertices);

    auto print_169 = std::bind(print_display_coords,_1,16.f,9.f);

    std::for_each(ndc_vertices.begin(),ndc_vertices.end(),print_169);

    return 0;
}
