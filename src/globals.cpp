#include "globals.h"
#include "shader.h"

using namespace shader;

//FBO


//Texture
GLuint textureID = 0;
bool firstTime = true;

//Projection and View Matrix
glm::mat4 projMatrix, viewMatrix;                    //transformation matrix
glm::mat3 normalMatrix;

//Splat's radii
float userRadiusFactor = 0.0125f;
float backupUserRadiusFactor;

//Camera
float cameraDistance = 4.0f;
glm::vec3 cameraEye = glm::vec3(0, 0, -cameraDistance);
glm::vec3 cameraUp = glm::vec3(0,1,0);
float cameraAngleX, cameraAngleY;

//Mouse
double lastMouseX = INT_MAX, lastMouseY = 0.0f; //last mouse position pressed;
bool leftBtnPress = false;

//Shaders
Shader fxaaFilter("FXAA",
                  "0_fxaa/vertexShader.glsl",
                  "0_fxaa/fragmentShader.glsl",
                  SINGLEPASS);


Shader sizedFixedShaderShader("Sized-Fixed Points",
                              "1_fixed-sized-points/vertexShader.glsl",
                              "1_fixed-sized-points/fragmentShader.glsl",
                              SINGLEPASS);

Shader squareSizedCorrectedShader("Square Size - Corrected by Depth",
                                  "2_square-Sized-corrected/vertexShader.glsl",
                                  "2_square-Sized-corrected/fragmentShader.glsl",
                                  SINGLEPASS);

Shader affinelyProjectedShader("Affinely Projected Point Sprites" ,
                               "3_affinely-projected-point-sprites/vertexShader.glsl",
                               "3_affinely-projected-point-sprites/fragmentShader.glsl",
                               SINGLEPASS);

Shader perspectiveCorrectedShaderMPVisibility("Gouraud",
                                              "4_perspective-corrected/pass_1_visibility/vertexShader.glsl",
                                              "4_perspective-corrected/pass_1_visibility/fragmentShader.glsl",
                                              DEPTH_MASK);

Shader perspectiveCorrectedShaderMPBlending("Gouraud",
                                            "4_perspective-corrected/pass_2_blending/gouraudVertexShader.glsl",
                                            "4_perspective-corrected/pass_2_blending/gouraudFragmentShader.glsl",
                                            BLENDING);

Shader perspectiveCorrectedShaderMPNormalization("Gouraud",
                                                 "4_perspective-corrected/pass_3_normalization/vertexShader.glsl",
                                                 "4_perspective-corrected/pass_3_normalization/fragmentShader.glsl",
                                                 NORMALIZATION);

static const Shader gouraud[] =   {perspectiveCorrectedShaderMPVisibility,
                                perspectiveCorrectedShaderMPBlending,
                                perspectiveCorrectedShaderMPNormalization};
vector<Shader> vecGouraud (gouraud, gouraud + sizeof(gouraud) / sizeof(gouraud[0]) );

Shader perspectiveCorrectedShaderPhongVisibility("Phong",
                                                 "4_perspective-corrected/pass_1_visibility/vertexShader.glsl",
                                                 "4_perspective-corrected/pass_1_visibility/fragmentShader.glsl",
                                                 DEPTH_MASK);

Shader perspectiveCorrectedShaderPhongBlending("Phong",
                                               "4_perspective-corrected/pass_2_blending/phongVertexShader.glsl",
                                               "4_perspective-corrected/pass_2_blending/phongFragmentShader.glsl",
                                               BLENDING);

Shader perspectiveCorrectedShaderPhongNormalization("Phong",
                                                    "4_perspective-corrected/pass_3_normalization/vertexShader.glsl",
                                                    "4_perspective-corrected/pass_3_normalization/fragmentShader.glsl",
                                                    NORMALIZATION);

static const Shader phong[] =   {perspectiveCorrectedShaderPhongVisibility,
                                 perspectiveCorrectedShaderPhongBlending,
                                 perspectiveCorrectedShaderPhongNormalization};
vector<Shader> vecPhong (phong, phong + sizeof(phong) / sizeof(phong[0]) );

static const vector<Shader> multipassArray[] = {vecGouraud, vecPhong};
vector<vector <Shader> > vec (multipassArray, multipassArray + sizeof(multipassArray) / sizeof(multipassArray[0]) );

Shader perspectiveCorrectedShader("Perspective Correct Rasterization",
                                  "4_perspective-corrected/vertexShader.glsl",
                                  "4_perspective-corrected/fragmentShader.glsl",
                                  SINGLEPASS,
                                  vec);

static const Shader arr2[] =   {sizedFixedShaderShader,
                                squareSizedCorrectedShader,
                                affinelyProjectedShader,
                                perspectiveCorrectedShader};

vector<Shader> vec2 (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]) );

string title;
unsigned int actualShader = 0;
unsigned int actualMultipass = 0;
vector<Shader> listOfShaders = vec2;

//int actualShader = 0;
bool MultipassEnabled = false;

bool FXAA = false;

bool colorEnabled = false;

bool automaticRadiusEnabled = false;

//Models
vector<struct vao> models;
unsigned int actualVAO = 0;

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

glm::vec3 normals[] = {
    normal_top,
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


