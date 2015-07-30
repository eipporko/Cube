#ifndef CUBE_globals_h
#define CUBE_globals_h

#include <vector>
#include "vao.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

class Shader;

extern GLuint textureID;
extern bool firstTime;

//Projection and View Matrix
extern glm::mat4 projMatrix, viewMatrix;                    //transformation matrix
extern glm::mat3 normalMatrix;

//Splat's radii
extern float userRadiusFactor;
extern float backupUserRadiusFactor;

//Camera
extern float cameraDistance;
extern glm::vec3 cameraEye;
extern glm::vec3 cameraUp;
extern float cameraAngleX, cameraAngleY;

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
extern vector<struct vao> models;
extern unsigned int actualVAO;

//Pointer to the VAO to be rendered in display func
extern struct vao* displayVAO;


//Cube Description
//
//    v1----v3
//   /|     /|
//  v2+---v4 |
//  | |    | |
//  | v5---+v7
//  |/     |/
//  v6----v8

extern struct vao cubeMesh;

#endif
