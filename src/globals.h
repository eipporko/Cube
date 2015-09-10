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
class Light;
class OrbitalLight;
class Camera;

class Globals {
private:
    Globals();
    
public:
    //Splat's radii
    static float userRadiusFactor;
    static float backupUserRadiusFactor;
    
    //Light
    static vector<vector<Light*> > sceneLightsList;
    static int sceneLightsArrIndex;
    
    //Window
    static string title;
    
    static Camera* mainCamera;
    
    //Mouse
    static double lastMouseX, lastMouseY; //last mouse position pressed;
    static bool leftBtnPress;
    
    //Shaders
    static Shader* fxaaFilter;
    static GLuint textureID;    //texture for renderToTexture in fxaa
    static bool firstTime;      //textureID initialized?
    static unsigned int actualShader;
    static unsigned int actualMultipass;
    static vector<Shader> listOfShaders;
    
    //Flags
    static bool MultipassEnabled;
    static bool FXAA;
    static bool colorEnabled;
    static bool automaticRadiusEnabled;
    
    //Models
    static vector<VAO> models;
    static unsigned int actualVAO;
    static VAO* displayVAO;     //Pointer to the VAO to be rendered in display func
    
    static void init();
    
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

extern VAO cubeMesh;

#endif
