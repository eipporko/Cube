#ifndef CUBE_globals_h
#define CUBE_globals_h

#include <vector>
#include "vao.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

class Shader;

//Shader program
extern GLint program;
extern GLuint f, v; //fragment and vertex shader

//Projection and View Matrix
extern GLint projMatrixLoc, viewMatrixLoc, normalMatrixLoc; //uniform locations
extern glm::mat4 projMatrix, viewMatrix;                    //transformation matrix
extern glm::mat3 normalMatrix;

//Frustrum and Viewport
extern GLint nearFrustumLoc, farFrustumLoc, topFrustumLoc, bottomFrustumLoc, leftFrustumLoc, rightFrustumLoc;
extern GLint hViewportLoc, wViewportLoc;

//Splat's radii
extern GLint radiusSplatLoc;
extern float radiusSplat;

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
extern string shaderDescription[];
extern int numOfShaders;
extern int actualShader;
extern vector<Shader*> shaders;

//int actualShader = 0;
extern bool MultipassEnabled;

//Models
extern vector<struct vao> models;
extern int actualVAO;

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
