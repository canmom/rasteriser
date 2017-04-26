#include <string>
#include <tclap/CmdLine.h>
#include <glm/vec3.hpp>

#include "arguments.h"

using std::string;
using glm::vec3;

Args::Args(int argc, char** argv) {
    try {
        TCLAP::CmdLine cmd("Render a model by rasterisation.", ' ');

        //file input
        TCLAP::ValueArg<std::string> objArg("o","obj","path to Wavefront .obj file containing model to render",false,"","model.obj",cmd);
        TCLAP::ValueArg<std::string> lightsArg("l","lights","path to CSV file containing directional lights in format direction_x,dir_y,dir_z,intensity,red,green,blue",true,"","lights.csv",cmd);

        //output parameters
        TCLAP::ValueArg<unsigned int> widthArg("x","width","Width of output in pixels",false,540u,"pixels",cmd);
        TCLAP::ValueArg<unsigned int> heightArg("y","height","Height of output in pixels",false,304u,"pixels",cmd);
        TCLAP::SwitchArg spinArg("s","spin","Display an animation of the model rotating around vertical (y) axis",cmd);
        TCLAP::SwitchArg flatArg("f","flat","Ignore vertex normals and use flat shading",cmd);
        TCLAP::SwitchArg windingArg("","wind-clockwise","Assume clockwise rather than anticlockwise winding angle to determine backfaces",cmd);

        //model transformations
        TCLAP::ValueArg<float> rotxArg("","rx","Rotate by angle around x axis (composed as YXZ Tait-Bryan angles).",false,0.f,"radians",cmd);
        TCLAP::ValueArg<float> rotyArg("","ry","Rotate by angle around y axis (composed as YXZ Tait-Bryan angles).",false,0.f,"radians",cmd);
        TCLAP::ValueArg<float> rotzArg("","rz","Rotate by angle around z axis (composed as YXZ Tait-Bryan angles).",false,0.f,"radians",cmd);
        TCLAP::ValueArg<float> scaleArg("m","scale","Scale the model by given factor",false,1.f,"factor",cmd);
        TCLAP::ValueArg<float> dispxArg("","dx","Displace model in x direction",false,0.f,"distance",cmd);
        TCLAP::ValueArg<float> dispyArg("","dy","Displace model in y direction",false,0.f,"distance",cmd);
        TCLAP::ValueArg<float> dispzArg("","dz","Displace model in z direction",false,0.f,"distance",cmd);
        
        cmd.parse(argc,argv);

        image_width = widthArg.getValue();
        image_height = heightArg.getValue();
        aspect_ratio = (float)image_width/(float)image_height;
        tait_bryan_angles = vec3(rotxArg.getValue(),rotyArg.getValue(),rotzArg.getValue());
        lights_file = lightsArg.getValue();
        obj_file = objArg.getValue();
        spin = spinArg.getValue();
        flat = flatArg.getValue();
        wind_clockwise = windingArg.getValue();
        scale = scaleArg.getValue();
        displacement = vec3(dispxArg.getValue(),dispyArg.getValue(),dispzArg.getValue());
    } catch (TCLAP::ArgException &e)  // catch any exceptions
    { std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl; exit(1);}
}