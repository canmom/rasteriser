//standard libraries
#include <vector>
#include <string>
#include <chrono>

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
        auto last_time = std::chrono::steady_clock::now();
        while(!window.is_closed()) {
            frame_buffer.fill(0);
            depth_buffer.fill(1.f);
            draw_frame(model_vertices, faces, lights, arguments, &frame_buffer, &depth_buffer);            
            window.display(frame_buffer);
            auto next_time = std::chrono::steady_clock::now();
            std::chrono::duration<float> time_step = next_time-last_time;
            last_time = next_time;
            arguments.angle += time_step.count();
        }
    }
    
    return 0;
}
