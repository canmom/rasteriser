#include <string>
#include <tclap/CmdLine.h>

#include "arguments.h"

using std::string;

Args::Args(int argc, char** argv) {
    try {
        TCLAP::CmdLine cmd("Render a model by rasterisation.", ' ');

        TCLAP::ValueArg<float> angleArg("a","camphi","Camera azimuthal view angle",false,0.f,"radians",cmd);
        TCLAP::ValueArg<unsigned int> widthArg("x","width","Width of output in pixels",false,540u,"pixels",cmd);
        TCLAP::ValueArg<unsigned int> heightArg("y","height","Height of output in pixels",false,304u,"pixels",cmd);
        TCLAP::ValueArg<std::string> objArg("o","obj","Wavefront .obj file to load",false,"null","model.obj",cmd);
        TCLAP::ValueArg<std::string> lightsArg("l","lights","CSV file containing directional lights in format direction_x,dir_y,dir_z,intensity,red,green,blue",true,"","lights.csv",cmd);
        TCLAP::SwitchArg spinArg("s","spin","Display an animation of the model rotating",cmd);
        
        cmd.parse(argc,argv);

        image_width = widthArg.getValue();
        image_height = heightArg.getValue();
        aspect_ratio = (float)image_width/(float)image_height;
        angle = angleArg.getValue();
        lights_file = lightsArg.getValue();
        obj_file = objArg.getValue();
        spin = spinArg.getValue();
    } catch (TCLAP::ArgException &e)  // catch any exceptions
    { std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl; exit(1);}
}