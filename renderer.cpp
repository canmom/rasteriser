//standard libraries
#include <vector>
#include <algorithm>
#include <functional>
#include <string>

//glm core libraries:
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>

//TCLAP
#include <tclap/CmdLine.h>

//libraries in local vendor folder
#include "CImg.h"

//project headers
#include "light.h"
#include "geometry.h"
#include "drawing.h"
#include "fileloader.h"

using std::vector;
using namespace std::placeholders;

using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::mat4;
using glm::uvec2;
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
    try {
        TCLAP::CmdLine cmd("Render a model by rasterisation.", ' ', "0.1");

        TCLAP::ValueArg<float> angleArg("a","camphi","Camera azimuthal view angle",false,0.f,"radians",cmd);
        TCLAP::ValueArg<unsigned int> widthArg("x","width","Width of output in pixels",false,540u,"pixels",cmd);
        TCLAP::ValueArg<unsigned int> heightArg("y","height","Height of output in pixels",false,304u,"pixels",cmd);
        TCLAP::ValueArg<std::string> objArg("o","obj","Wavefront .obj file to load",false,"null","file.obj",cmd);
        TCLAP::ValueArg<float> lightThetaArg("t","lighttheta","inclination angle of main light",false,1.f,"radians",cmd);
        TCLAP::ValueArg<float> lightPhiArg("p","lightphi","azimuth angle of main light",false,1.f,"radians",cmd);
        
        cmd.parse(argc,argv);

        //define output width and height
        unsigned int image_width = widthArg.getValue();
        unsigned int image_height = heightArg.getValue();
        float aspect_ratio = (float)image_width/(float)image_height;
        float angle = angleArg.getValue();
        float lightTheta = lightThetaArg.getValue();
        float lightPhi = lightPhiArg.getValue();

        vec3 lightDir(-glm::sin(lightTheta)*glm::cos(lightPhi),-glm::cos(lightTheta),-glm::sin(lightTheta)*glm::sin(lightPhi));


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

        //add a light as defined by user
        lights.push_back(Light(lightDir,500.f,vec3(1.f)));

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
        for (auto face = faces.begin(); face < faces.end(); ++face) {
            draw_triangle(*face,raster_vertices,camera_vertices,lights,&frame_buffer,&depth_buffer,image_width,image_height);
        }

        // auto dt = std::bind(draw_triangle, _1, raster_vertices, camera_vertices, lights, &frame_buffer, &depth_buffer, image_width, image_height);
        // std::for_each(faces.begin(),faces.end(),dt);

        //output frame and depth buffers
        frame_buffer.save("frame.png");
        depth_buffer.normalize(0,255).save("depth.png");
        
        return 0;

    } catch (TCLAP::ArgException &e)  // catch any exceptions
    { std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl; }
}
