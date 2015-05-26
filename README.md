Cube
========================

Test it and you could see an awesome colored cube in your screen.
Using an incredible 3D technology which will allow you to navigate around the cube!. 

it's Epic!


Controls
-------------------------
* O: Open .PTS or .PLY files
* Mouse: Move camera
* Scroll: Zoom
* Up/Down: Change splat's radii
* R: Reset camera position
* S: Switch between shaders
* M: Switch between models (CUBE | SPHERE)
* P: Change between single & multi-pass rendering.
* Esc: Exit


What do you need to build your own Cube
----------------------------------------
In order to build Cube, you need to have:
* [gcc & g++](http://gcc.gnu.org/) or [Clang](http://clang.llvm.org/) compiling and linking C, C++ (if clang, need at least v5.0, Xcode 5.0.1 on OS X)
* OpenGL 4 support.
* [cmake](http://www.cmake.org/) 2.8 or later. 
* [git](http://git-scm.com/) 1.8.3.4 or later. 
* [GLFW](http://www.glfw.org/) Library 3.0.3 or later.
* [GLEW](http://glew.sourceforge.net/) Library 1.10.0 or later.
* [GLM](http://glm.g-truc.net/) Library 0.9.4 or later.
* [PCL](http://pointclouds.org/) Library 1.3 or later.

(Earlier versions might work OK, but are not tested.)

### Mac Os X - Only Mavericks
Mac Os users should install the following components:
* Apple's [Xcode](https://developer.apple.com/technologies/tools/) Developer Tools (version 5.0.1 or later) 
* Apple's Command Line Developer Tools are found at the [Apple Developer](https://developer.apple.com/downloads/) site, or they can be installed from within Xcode.
* [Macports](http://www.macports.org/). Once Macports is installed, run `sudo port selfupdate` to update macports repositories. Then run `sudo port install glew` and `sudo port install glm` for install the dependencies.
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
  sudo port install boost
  sudo port install eigen3
  sudo port install flann
  sudo port install vtk5 +qt4_mac

  git clone https://github.com/PointCloudLibrary/pcl.git
  cd pcl
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  ```

How to build
-------------------------------
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



Platform-Specific Build Notes
--------------------------------

### Mac Os X
If you want to create Xcode project files, you only have to run `cmake . -G Xcode`. 
This approach is highly recommended to do it from an out-of-source build directory. *- explained in 'How to build' step*
