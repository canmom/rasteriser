//standard libraries
#include <vector>
#include <string>
#include <chrono>
#include <string>

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

using std::vector;

using glm::vec3;
using glm::vec4;
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

int main(int argc,char** argv) {
    Args arguments(argc,argv);    

    //define storage for vertices
    vector<vec3> model_vertices;

    //define storage for faces (indices into vertices)
    vector<uvec3> faces;

    //define storage for lights
    vector<Light> lights;

    //add lights as defined by user
    load_lights(arguments.lights_file,lights);

    //load model to render
    if (arguments.obj_file!="null") {
        load_obj(arguments.obj_file,model_vertices,faces);
    } else {
        //load dummy data
        add_square(model_vertices,faces);
    }

    //instantiate buffers for storing pixel data
    CImg<unsigned char> frame_buffer(arguments.image_width,arguments.image_height,1,3,0);
    CImg<float> depth_buffer(arguments.image_width,arguments.image_height,1,1,1.f);

    if (not arguments.spin) {
        draw_frame(model_vertices, faces, lights, arguments, &frame_buffer, &depth_buffer);

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
            draw_frame(model_vertices, faces, lights, arguments, &frame_buffer, &depth_buffer);

            //display the frame rate
            frame_buffer.draw_text(5,5,frame_rate.c_str(),white,black);
            window.display(frame_buffer);

            //calculate the time elapsed drawing
            auto next_time = std::chrono::steady_clock::now();
            time_step = next_time-last_time;
            frame_rate = std::to_string(1.f/time_step.count());
            last_time = next_time;

            //rotate proportional to time elapsed
            arguments.angle += time_step.count();
        }
    }
    
    return 0;
}
