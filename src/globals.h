#ifndef CUBE_globals_h
#define CUBE_globals_h

#include <vector>
#include "vao.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <glm/gtc/type_ptr.hpp>

#define LIGHT_DISTANCE 4.0f

using namespace std;

class Shader;
class OrbitalLight;
class Camera;

extern GLuint textureID;
extern bool firstTime;

//Splat's radii
extern float userRadiusFactor;
extern float backupUserRadiusFactor;

//Camera
extern Camera orbitalCamera;

//Light
extern bool orbitalLightEnabled;
extern vector<vector<OrbitalLight*> > sceneLightsList;
extern int sceneLightsArrIndex;
//extern OrbitalLight* cameraLight;
//extern vector<OrbitalLight*> orbitalLightsList;

//Mouse
extern double lastMouseX, lastMouseY; //last mouse position pressed;
extern bool leftBtnPress;

//Shaders
extern string title;
extern unsigned int actualShader;
extern unsigned int actualMultipass;
extern vector<Shader> listOfShaders;

extern Shader fxaaFilter;

//int actualShader = 0;
extern bool MultipassEnabled;

extern bool FXAA;

extern bool colorEnabled;

extern bool automaticRadiusEnabled;

//Models
extern vector<VAO> models;
extern unsigned int actualVAO;

//Pointer to the VAO to be rendered in display func
extern VAO* displayVAO;


//Cube Description
//
//    v1----v3
//   /|     /|
//  v2+---v4 |
//  | |    | |
//  | v5---+v7
//  |/     |/
//  v6----v8

extern VAO cubeMesh;

#endif
