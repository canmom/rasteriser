//standard libraries
#include <vector>
#include <string>
#include <chrono>
#include <array>

//glm core libraries:
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

//libraries in local vendor folder
#include "CImg.h"

//project headers
#include "light.h"
#include "geometry.h"
#include "drawing.h"
#include "fileloader.h"
#include "arguments.h"
#include "face.h"
#include "material.h"

using std::vector;

using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::uvec3;

using cimg_library::CImg;

void add_square(vector<vec3> &vertices, vector<Triangle> &faces,vector<vec3> &vertnormals,vector<vec2> &vertuvs, vector<Material> & materials) {
    //Load a very simple scene for testing
    vertices.push_back(vec3(-0.5f,-0.5f,0.0f));
    vertices.push_back(vec3(0.5f,-0.5f,0.0f));
    vertices.push_back(vec3(-0.5f,0.5f,0.0f));
    vertices.push_back(vec3(0.5f,0.5f,0.0f));

    vertnormals.push_back(vec3(0.f,0.f,1.f));

    vertuvs.push_back(vec2(0.f,0.f));
    vertuvs.push_back(vec2(1.f,0.f));
    vertuvs.push_back(vec2(0.f,1.f));
    vertuvs.push_back(vec2(1.f,1.f));

    faces.push_back(Triangle({0,1,2},{0,0,0},{0,1,2},0));
    faces.push_back(Triangle({2,1,3},{0,0,0},{2,1,3},0));

    materials.push_back(Material(vec3(1.f)));
}

int main(int argc,char** argv) {
    Args arguments(argc,argv);    

    //define storage for vertices
    vector<vec3> model_vertices;

    //define storage for vertex normals
    vector<vec3> model_vertnormals;

    //define storage for UV coordinates
    vector<vec2> vertuvs;

    //define storage for faces (indices into vertices)
    vector<Triangle> faces;

    //define storage for materials
    vector<Material> materials;

    //define storage for lights
    vector<Light> lights;

    //add lights as defined by user
    load_lights(arguments.lights_file,lights);

    //load model to render
    if (not arguments.obj_file.empty()) {
        load_obj(arguments,model_vertices,faces,model_vertnormals, vertuvs, materials);
    } else {
        //load a square if no model provided
        add_square(model_vertices,faces,model_vertnormals,vertuvs, materials);
    }

    //instantiate buffers for storing pixel data
    CImg<unsigned char> frame_buffer(arguments.image_width,arguments.image_height,1,3,0);
    CImg<float> depth_buffer(arguments.image_width,arguments.image_height,1,1,1.f);

    if (not arguments.spin) {
        draw_frame(model_vertices, faces, model_vertnormals, vertuvs, lights, materials, arguments, &frame_buffer, &depth_buffer);

        //output frame and depth buffers
        frame_buffer.save("frame.png");
        depth_buffer.normalize(0,255).save("depth.png");
    } else {
        cimg_library::CImgDisplay window(frame_buffer,"Render");

        //initialise values for drawing time step
        auto last_time = std::chrono::steady_clock::now();
        std::string frame_rate;
        std::chrono::duration<float> time_step;
        unsigned char white[3] = {255,255,255};
        unsigned char black[3] = {0,0,0};

        //drawing loop
        while(!window.is_closed()) {
            //clear the frame
            frame_buffer.fill(0);
            depth_buffer.fill(1.f);

            //render
            draw_frame(model_vertices, faces, model_vertnormals, vertuvs, lights, materials, arguments, &frame_buffer, &depth_buffer);

            //display the frame rate
            frame_buffer.draw_text(5,5,frame_rate.c_str(),white,black);
            window.display(frame_buffer);

            //calculate the time elapsed drawing
            auto next_time = std::chrono::steady_clock::now();
            time_step = next_time-last_time;
            frame_rate = std::to_string(1.f/time_step.count());
            last_time = next_time;

            //rotate proportional to time elapsed
            arguments.tait_bryan_angles.y += time_step.count();
        }
    }
    
    return 0;
}
