/*
 *
 * CUBE
 *
 * Copyright (c) David Antunez Gonzalez 2013-2015 <dantunezglez@gmail.com>
 * Copyright (c) Luis Omar Alvarez Mures 2013-2015 <omar.alvarez@udc.es>
 * Copyright (c) Emilio Padron Gonzalez 2013-2015 <emilioj@gmail.com>
 *
 * All rights reserved.
 *
 * This file is part of ToView.
 *
 * CUBE is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library.
 *
 */

#include "globals.h"
#include "shader.h"
#include "light.h"
#include "orbitallight.h"
#include "cameralight.h"
#include "staticlight.h"
#include "camera.h"

using namespace shader;


float Globals::userRadiusFactor;
float Globals::backupUserRadiusFactor;
vector<vector<Light*> > Globals::sceneLightsList;
int Globals::sceneLightsArrIndex;
string Globals::title;
Camera* Globals::mainCamera;
double Globals::lastMouseX, Globals::lastMouseY;
bool Globals::leftBtnPress;
Shader* Globals::fxaaFilter;
GLuint Globals::textureID;
bool Globals::firstTime;
unsigned int Globals::actualShader;
unsigned int Globals::actualMultipass;
vector<Shader> Globals::listOfShaders;
bool Globals::MultipassEnabled;
bool Globals::FXAA;
bool Globals::colorEnabled;
bool Globals::automaticRadiusEnabled;
vector<VAO> Globals::models;
unsigned int Globals::actualVAO;
VAO* Globals::displayVAO;

void Globals::init() {
    
    //Splat's radii
    userRadiusFactor = 0.0125f;
    
    
    mainCamera = new Camera(glm::vec3(0, 0, -4.0f));
    mainCamera->setActive();
    
    
    //Lights
    vector<Light*> noneLights;
    
    vector<Light*> cameraLights;
    cameraLights.push_back(new CameraLight(Camera::activeCamera, glm::vec3 (1,1,1), 1.f));
    
    vector<Light*> orbitalWhiteLights;
    orbitalWhiteLights.push_back(new OrbitalLight(glm::vec3(0, 0, -LIGHT_DISTANCE), glm::vec3(1,1,1), 0.25f, glm::vec3(0,1,0), 5.f ));
    
    vector<Light*> axisOrbitalLights;
    axisOrbitalLights.push_back(new OrbitalLight(glm::vec3(0, 0, LIGHT_DISTANCE), glm::vec3(0,1,0), 0.30f, glm::vec3(0,1,0), 5.0f )) ;
    axisOrbitalLights.push_back(new OrbitalLight(glm::vec3(0, LIGHT_DISTANCE, 0), glm::vec3(1,0,0), 0.30f, glm::vec3(1,0,0), 2.5f ));
    axisOrbitalLights.push_back(new OrbitalLight(glm::vec3(LIGHT_DISTANCE, 0, 0), glm::vec3(0,0,1), 0.50f, glm::vec3(0,0,1), 1.25f ));
    
    vector<Light*> mixedLights;
    mixedLights.push_back(new OrbitalLight(glm::vec3(0, 0, LIGHT_DISTANCE), glm::vec3(0,1,0), 0.30f, glm::vec3(0,1,0), 5.0f )) ;
    mixedLights.push_back(new OrbitalLight(glm::vec3(0, LIGHT_DISTANCE, 0), glm::vec3(1,0,0), 0.30f, glm::vec3(1,0,0), 2.5f ));
    mixedLights.push_back(new OrbitalLight(glm::vec3(LIGHT_DISTANCE, 0, 0), glm::vec3(0,0,1), 0.30f, glm::vec3(0,0,1), 1.25f ));
    mixedLights.push_back(new StaticLight(glm::vec3(0, 0, LIGHT_DISTANCE), glm::vec3(1,1,1), 0.30f));
    mixedLights.push_back(new StaticLight(glm::vec3(0, 0, -LIGHT_DISTANCE), glm::vec3(1,1,1), 0.30f));
    
    vector<Light*> sevenLights;
    sevenLights.push_back(new OrbitalLight(glm::vec3(0, 0, LIGHT_DISTANCE), glm::vec3(0,1,0), 0.30f, glm::vec3(0,1,0), 5.0f )) ;
    sevenLights.push_back(new OrbitalLight(glm::vec3(0, LIGHT_DISTANCE, 0), glm::vec3(1,0,0), 0.30f, glm::vec3(1,0,0), 2.5f ));
    sevenLights.push_back(new OrbitalLight(glm::vec3(LIGHT_DISTANCE, 0, 0), glm::vec3(0,0,1), 0.30f, glm::vec3(0,0,1), 1.25f ));
    sevenLights.push_back(new StaticLight(glm::vec3(0, 0, LIGHT_DISTANCE), glm::vec3(1,1,1), 0.30f));
    sevenLights.push_back(new StaticLight(glm::vec3(0, 0, -LIGHT_DISTANCE), glm::vec3(1,1,1), 0.30f));
    sevenLights.push_back(new StaticLight(glm::vec3(-LIGHT_DISTANCE, 0, 0), glm::vec3(1,1,0), 0.30f));
    sevenLights.push_back(new StaticLight(glm::vec3(LIGHT_DISTANCE, 0, 0), glm::vec3(0,1,1), 0.30f));
    
    vector<Light*> nineLights;
    nineLights.push_back(new OrbitalLight(glm::vec3(0, 0, LIGHT_DISTANCE), glm::vec3(0,1,0), 0.30f, glm::vec3(0,1,0), 5.0f )) ;
    nineLights.push_back(new OrbitalLight(glm::vec3(0, LIGHT_DISTANCE, 0), glm::vec3(1,0,0), 0.30f, glm::vec3(1,0,0), 2.5f ));
    nineLights.push_back(new OrbitalLight(glm::vec3(LIGHT_DISTANCE, 0, 0), glm::vec3(0,0,1), 0.30f, glm::vec3(0,0,1), 1.25f ));
    nineLights.push_back(new StaticLight(glm::vec3(0, 0, LIGHT_DISTANCE), glm::vec3(1,1,1), 0.30f));
    nineLights.push_back(new StaticLight(glm::vec3(0, 0, -LIGHT_DISTANCE), glm::vec3(1,1,1), 0.30f));
    nineLights.push_back(new StaticLight(glm::vec3(-LIGHT_DISTANCE, 0, 0), glm::vec3(1,1,0), 0.30f));
    nineLights.push_back(new StaticLight(glm::vec3(LIGHT_DISTANCE, 0, 0), glm::vec3(0,1,1), 0.30f));
    nineLights.push_back(new StaticLight(glm::vec3(0, -LIGHT_DISTANCE, 0), glm::vec3(1,0,1), 0.30f));
    nineLights.push_back(new StaticLight(glm::vec3(0, LIGHT_DISTANCE, 0), glm::vec3(0,1,0), 0.30f));
    
    
    
    sceneLightsList.push_back(noneLights);
    sceneLightsList.push_back(orbitalWhiteLights);
    sceneLightsList.push_back(cameraLights);
    sceneLightsList.push_back(axisOrbitalLights);
    sceneLightsList.push_back(mixedLights);
    sceneLightsList.push_back(sevenLights);
    sceneLightsList.push_back(nineLights);
    sceneLightsArrIndex = 0;
    
    
    //Mouse
    lastMouseX = INT_MAX, lastMouseY = 0.0f; //last mouse position pressed;
    leftBtnPress = false;
    
    
    //Shaders
    fxaaFilter = new Shader("FXAA",
                            "0_fxaa/vertexShader.glsl",
                            "0_fxaa/fragmentShader.glsl",
                            SINGLEPASS);
    textureID = 0;
    firstTime = true;
    actualShader = 0;
    actualMultipass = 0;
    
    listOfShaders.push_back(Shader("Sized-Fixed Points",
                                    "1_fixed-sized-points/vertexShader.glsl",
                                    "1_fixed-sized-points/fragmentShader.glsl",
                                    SINGLEPASS) );
    listOfShaders.push_back(Shader("Square Size - Corrected by Depth",
                                   "2_square-Sized-corrected/vertexShader.glsl",
                                   "2_square-Sized-corrected/fragmentShader.glsl",
                                   SINGLEPASS));
    listOfShaders.push_back(Shader("Affinely Projected Point Sprites" ,
                                   "3_affinely-projected-point-sprites/vertexShader.glsl",
                                   "3_affinely-projected-point-sprites/fragmentShader.glsl",
                                   SINGLEPASS));
    vector<Shader> gouraud;
    gouraud.push_back(Shader("Gouraud",
                             "4_perspective-corrected/pass_1_visibility/vertexShader.glsl",
                             "4_perspective-corrected/pass_1_visibility/fragmentShader.glsl",
                             DEPTH_MASK));
    gouraud.push_back(Shader("Gouraud",
                             "4_perspective-corrected/pass_2_blending/gouraudVertexShader.glsl",
                             "4_perspective-corrected/pass_2_blending/gouraudFragmentShader.glsl",
                             BLENDING));
    gouraud.push_back(Shader("Gouraud",
                             "4_perspective-corrected/pass_3_normalization/vertexShader.glsl",
                             "4_perspective-corrected/pass_3_normalization/fragmentShader.glsl",
                             NORMALIZATION));
    
    vector<Shader> phong;
    phong.push_back(Shader("Phong",
                           "4_perspective-corrected/pass_1_visibility/vertexShader.glsl",
                           "4_perspective-corrected/pass_1_visibility/fragmentShader.glsl",
                           DEPTH_MASK));
    phong.push_back(Shader("Phong",
                           "4_perspective-corrected/pass_2_blending/phongVertexShader.glsl",
                           "4_perspective-corrected/pass_2_blending/phongFragmentShader.glsl",
                           BLENDING));
    phong.push_back(Shader("Phong",
                           "4_perspective-corrected/pass_3_normalization/vertexShader.glsl",
                           "4_perspective-corrected/pass_3_normalization/fragmentShader.glsl",
                           NORMALIZATION));
    
    vector<Shader> deferred;
    deferred.push_back(Shader("Deferred",
                              "4_perspective-corrected/pass_1_visibility/vertexShader.glsl",
                              "4_perspective-corrected/pass_1_visibility/fragmentShader.glsl",
                              DEPTH_MASK));
    deferred.push_back(Shader("Deferred",
                              "4_perspective-corrected/pass_2_blending/deferredVertexShader.glsl",
                              "4_perspective-corrected/pass_2_blending/deferredFragmentShader.glsl",
                              BLENDING));
    deferred.push_back(Shader("Deferred",
                              "4_perspective-corrected/pass_3_normalization/deferredVertexShader.glsl",
                              "4_perspective-corrected/pass_3_normalization/deferredFragmentShader.glsl",
                              NORMALIZATION));
    
    vector<vector <Shader> > vec;
    vec.push_back(gouraud);
    vec.push_back(phong);
    vec.push_back(deferred);
    
    listOfShaders.push_back(Shader("Perspective Correct Rasterization",
                                   "4_perspective-corrected/vertexShader.glsl",
                                   "4_perspective-corrected/fragmentShader.glsl",
                                   SINGLEPASS,
                                   vec) );
    
    textureID = 0;
    firstTime = true;
    
    //Flags
    MultipassEnabled = false;
    FXAA = false;
    colorEnabled = false;
    automaticRadiusEnabled = false;
    
    //Models
    actualVAO = 0;
    displayVAO = NULL;
    
};

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

VAO cubeMesh(36,
             12,
             vector<glm::vec3> (vertices, vertices + sizeof(vertices)/sizeof(glm::vec3)),
             vector<glm::vec3> (colors, colors + sizeof(colors)/sizeof(glm::vec3)),
             vector<glm::vec3> (normals, normals + sizeof(normals)/sizeof(glm::vec3)),
             GL_TRIANGLES);


