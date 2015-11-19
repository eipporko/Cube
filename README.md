# Cube [![Build Status](https://travis-ci.org/eipporko/Cube.svg?branch=master)](https://travis-ci.org/eipporko/Cube)

Test it and you could see an awesome colored cube in your screen.
Using an incredible 3D technology which will allow you to navigate around the cube!.

it's Epic!


## Controls

* Mouse: Move camera
* Scroll: Zoom
* Up/Down: Change splat's radii
* A: Automatic Variable Splat Radius / User Uniform Splat radius
* C: RGB/NONE
* F: Activate/Deactivate FXAA
* L: Switch between differents set of lights (Only on Perspective Correct mode).
* M: Switch between models  (CUBE | SPHERE | Opened Models)
* O: Open .PCD or .PLY files
* P: Change between Flat, Gouraud, Phong & Deferred Shading(Only on Perspective Correct mode).
* Q: Recompile the actual shader.
* R: Reset camera position
* S: Switch between shaders (Sized-Fixed | Corrected by Depth | Affinely Projected Sprites | Perspective Correct)
* Esc: Exit


## What do you need to build your own Cube

In order to build Cube, you need to have:
* [gcc & g++](http://gcc.gnu.org/) or [Clang](http://clang.llvm.org/) compiling and linking C, C++ (if clang, need at least v5.0, Xcode 5.0.1 on OS X)
* OpenGL 4 support.
* [cmake](http://www.cmake.org/) 2.8 or later.
* [git](http://git-scm.com/) 1.8.3.4 or later.
* [GLFW](http://www.glfw.org/) Library 3.0.3 or later.
* [GLEW](http://glew.sourceforge.net/) Library 1.10.0 or later.
* [GLM](http://glm.g-truc.net/) Library 0.9.6 or later.
* [PCL](http://pointclouds.org/) Library 1.3 or later.

(Earlier versions might work OK, but are not tested.)

### Mac Os X - Mavericks or earlier
Mac Os users should install the following components:
* Apple's [Xcode](https://developer.apple.com/technologies/tools/) Developer Tools (version 5.0.1 or later)
* Apple's Command Line Developer Tools are found at the [Apple Developer](https://developer.apple.com/downloads/) site, or they can be installed from within Xcode.
* [Homebrew](http://brew.sh/). Once Homebrew is installed, run `brew update` to update brew repositories. Then run `brew install glew` and `brew install glm` for install the dependencies.
* To compile and install [GLFW](http://www.glfw.org/) libs, you can clone the official [repository](https://github.com/glfw/glfw) and follow the [guide](http://www.glfw.org/docs/latest/compile.html) in the GLFW documentation.

  ```
  git clone https://github.com/glfw/glfw.git
  cd glfw
  cmake . -DGLFW_BUILD_UNIVERSAL=ON
  make
  sudo make install
  ```
* To compile and install [PCL](http://pointclouds.org/) libs, you can clone the official [repository](https://github.com/PointCloudLibrary/pcl) and follow the [guide](http://pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php). Make sure you have installed Boost, Eigen, FLANN and Visualization ToolKit (VTK) dependencies.

  ```
  brew tap homebrew/science
  brew install boost
  brew install eigen
  brew install flann
  brew install vtk 
  brew install qt

  git clone https://github.com/PointCloudLibrary/pcl.git
  cd pcl
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  ```

### Ubuntu 14.04 LTS
Ubuntu users should install the following components:

* It's possible to install [GLEW](http://glew.sourceforge.net/) from Ubuntu repositories.
  ```
  sudo apt-get update && sudo apt-get upgrade
  sudo apt-get install libglew-dev
  ```

* To compile and install [GLM](http://glm.g-truc.net/) lib, you can clone the official [repository](https://github.com/g-truc/glm.git).

  ```
  git clone https://github.com/g-truc/glm.git
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  ```

* To compile and install [GLFW](http://www.glfw.org/) libs, you can clone the official [repository](https://github.com/glfw/glfw) and follow the [guide](http://www.glfw.org/docs/latest/compile.html) in the GLFW documentation. Make sure you have installed xorg-dev and libgl1-mesa-dev packages.

  ```
  sudo apt-get install xorg-dev
  sudo apt-get install libgl1-mesa-dev

  git clone https://github.com/glfw/glfw.git
  cd glfw
  cmake . -BUILD_SHARED_LIBS = ON
  make
  sudo make install
  ```
  > NVIDIA GRAPHIC CARDS: Remember overwrote the symbolic link in '/usr/lib/x86_64-linux-gnu/libGL.so' with the nvidia openGL library before build GLFW libs. ( Delete the old symbolic link before do this ).

* To compile and install [PCL](http://pointclouds.org/) libs, you can clone the official [repository](https://github.com/PointCloudLibrary/pcl). Make sure you have installed Boost, Eigen, FLANN and Visualization ToolKit (VTK) dependencies.

  ```
  sudo apt-get install libeigen3-dev
  sudo apt-get install libflann-dev
  sudo apt-get install libboost-all-dev
  sudo apt-get install libvtk5.8-qt4 libvtk5.8 libvtk5-dev

  git clone https://github.com/PointCloudLibrary/pcl.git
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  ```

## How to build
For Linux/Unix/OSX/Mingw/Cygwin the build is straight forward:
```
git clone https://github.com/eipporko/Cube.git
cd Cube
cmake .
make
```

Alternatively, you can create an out-of-source build directory and run cmake from there. The advantage to this approach is that the
temporary files created by CMake won't clutter the Cube source directory, and also makes it possible to have multiple
independent build targets by creating multiple build directories. In adirectory alongside the Cube use:

```
git clone https://github.com/eipporko/Cube.git
cd Cube
mkdir build
cd build
cmake ..
make
```

If you have a multi-processor/core system then you can use make -j <numcores> to tell the build system to multiple cores.



## Platform-Specific Build Notes

### Mac Os X
If you want to create Xcode project files, you only have to run `cmake . -G Xcode`.
This approach is highly recommended to do it from an out-of-source build directory. *- explained in 'How to build' step*

## Analysis Tools

When cube is in debug mode, at the end of each session saves information in a log file. For analysis purposes Cube bring tools for make easier to understand the log, building graphs which compares the different rendering times achieved.

### Dependencies

#### Ubuntu 14.04 LTS

  ```
  sudo apt-get install python-setuptools
  sudo apt-get install python-scitools

  git clone git://github.com/pudo/dataset.git
  cd dataset/
  sudo python setup.py install
  ```

### How to work with
```
$ python cubeGraphGen.py

CUBE GRAPH GENERATOR

USAGE: cubeGraphGen.py [options] logFile

OPTIONS:
    -h        Display available options
    --help    Display available options
    -v        Run in verbose mode
May the Force be with you :)
```
