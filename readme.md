# Software rasteriser

An implementation of the rasterisation algorithm for drawing 3D triangles using OpenGL-imitation maths.

This is a self-learning project intended to teach me how an Open-GL style renderer works behind the scenes. It is fairly closely based on the approach described at [Scratchapixel](https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation/overview-rasterization-algorithm), but with some modifications to use the OpenGL perspective projection matrix. A log of development can be found [http://canonicalmomentum.tumblr.com/tagged/building-my-own-rasteriser/chrono](here).

## Third party code

Includes the [CImg](http://cimg.eu/) library. The license is contained in the vendor/cimg/ folder along with the header file.

Needed for compilation:
- [OpenGL Mathematics (GLM)](https://github.com/g-truc/glm), which should be installed using [Conan](https://www.conan.io/) as described below.
- [CMake](https://cmake.org/).

Needed to run:
- [ImageMagick](http://imagemagick.org/script/index.php) or [GraphicsMagick](http://imagemagick.org/script/index.php) to allow CImg to output PNG files.

## Building the project

1. Create a `build` folder inside the project folder.
2. In the build folder, run `conan install ..` to install GLM via Conan, and produce a CMake file.
3. Run `cmake ..` to produce build files appropriate to your build system (e.g. a makefile for GCC).
4. Compile according to your system (e.g. with `make` for gcc, or `ming32-make` for MinGW-GCC).

##Using the project
For now, there are no command-line parameters or file input. Running the binary file will produce two images, 'frame.png' containing the rendered output, and 'depth.png' containing the (normalised) depth buffer.