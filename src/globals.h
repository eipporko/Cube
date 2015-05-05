#ifndef CUBE_globals_h
#define CUBE_globals_h

#include <vector>
#include "vao.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

//Shader program
GLint program;
GLuint f, v; //fragment and vertex shader

//Projection and View Matrix
GLint projMatrixLoc, viewMatrixLoc, normalMatrixLoc; //uniform locations
glm::mat4 projMatrix, viewMatrix;                    //transformation matrix
glm::mat3 normalMatrix;

//Frustrum and Viewport
GLint nearFrustumLoc, farFrustumLoc, topFrustumLoc, bottomFrustumLoc, leftFrustumLoc, rightFrustumLoc;
GLint hViewportLoc, wViewportLoc;

//Splat's radii
GLint radiusSplatLoc;
float radiusSplat = 0.0125f;

//Camera
float cameraDistance = 4.0f;
glm::vec3 cameraEye = glm::vec3(0, 0, -cameraDistance);
glm::vec3 cameraUp = glm::vec3(0,1,0);
float cameraAngleX, cameraAngleY;

//Mouse
double lastMouseX = INT_MAX, lastMouseY = 0.0f; //last mouse position pressed;
bool leftBtnPress = false;

//Shaders
string shaderDescription[] = {  "Sized-Fixed Points",
                                "Square Size - Corrected by Depth",
                                "Affinely Projected Point Sprites" ,
                                "Perspective Correct Rasterization"};
int numOfShaders = sizeof(shaderDescription)/sizeof(string);
int actualShader = numOfShaders - 1;


//Models
vector<struct vao> models;
int actualVAO = 0;

//Pointer to the VAO to be rendered in display func
struct vao* displayVAO = NULL;

//Cube Description
//
//    v1----v3
//   /|     /|
//  v2+---v4 |
//  | |    | |
//  | v5---+v7
//  |/     |/
//  v6----v8

glm::vec3 v1 = glm::vec3(-1,1,1);
glm::vec3 v2 = glm::vec3(-1,1,-1);
glm::vec3 v3 = glm::vec3(1,1,1);
glm::vec3 v4 = glm::vec3(1,1,-1);
glm::vec3 v5 = glm::vec3(-1,-1,1);
glm::vec3 v6 = glm::vec3(-1,-1,-1);
glm::vec3 v7 = glm::vec3(1,-1,1);
glm::vec3 v8 = glm::vec3(1,-1,-1);

glm::vec3 blue  =   glm::vec3(0,0,1);
glm::vec3 green =   glm::vec3(0,1,0);
glm::vec3 cyan  =   glm::vec3(0,1,1);
glm::vec3 red   =   glm::vec3(1,0,0);
glm::vec3 magenta = glm::vec3(1,0,1);
glm::vec3 yellow =  glm::vec3(1,1,0);

glm::vec3 normal_top = glm::vec3(0,1,0);
glm::vec3 normal_front = glm::vec3(0,0,-1);
glm::vec3 normal_right = glm::vec3(1,0,0);
glm::vec3 normal_bottom = glm::vec3(0,-1,0);
glm::vec3 normal_left = glm::vec3(-1,0,0);
glm::vec3 normal_back = glm::vec3(0,0,1);

glm::vec3 vertices[] = {v1, v2, v3, //Top
    v3, v2, v4,
    
    v4, v2, v6, //Front
    v6, v8, v4,
    
    v4, v8, v3, //Right
    v3, v8, v7,
    
    v7, v8, v5, //Bottom
    v5, v8, v6,
    
    v6, v2, v5, //Left
    v5, v2, v1,
    
    v1, v3, v7, //Back
    v7, v5, v1 };

glm::vec3 colors[] = {  blue, blue, blue,   //Top
    blue, blue, blue,
    
    green, green, green,  //Front
    green, green, green,
    
    cyan, cyan, cyan,   //Right
    cyan, cyan, cyan,
    
    red, red, red,    //Bottom
    red, red, red,
    
    magenta, magenta, magenta,//Left
    magenta, magenta, magenta,
    
    yellow, yellow, yellow, //Back
    yellow, yellow, yellow };

glm::vec3 normals[] = { normal_top,
    normal_top,
    
    normal_front,
    normal_front,
    
    normal_right,
    normal_right,
    
    normal_bottom,
    normal_bottom,
    
    normal_left,
    normal_left,
    
    normal_back,
    normal_back };


struct vao cubeMesh = {
    .vaoID = 0,
    .vboID = 0,
    .numOfVertices = 36,
    .numOfTriangles = 12,
    .vertices = vector<glm::vec3> (vertices, vertices + sizeof(vertices)/sizeof(glm::vec3)),
    .colors = vector<glm::vec3> (colors, colors + sizeof(colors)/sizeof(glm::vec3)),
    .normals = vector<glm::vec3> (normals, normals + sizeof(normals)/sizeof(glm::vec3)),
    .mode = GL_TRIANGLES
};


#endif
