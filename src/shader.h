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

#ifndef CUBE_shader_h
#define CUBE_shader_h

#include <iostream>
#include <vector>

#include <GL/glew.h>

#define PATH_TO_SHADERS "../src/shaders/"

namespace shader
{
    enum shaderMode {
        SINGLEPASS      = 0,
        DEPTH_MASK      = 1,
        BLENDING        = 2,
        NORMALIZATION   = 3
    };
}

using namespace std;
using namespace shader;

class Shader
{
private:
    //variables
    string description;
    string vertexShaderPath;
    string fragmentShaderPath;
    shaderMode mode;
    vector<vector<Shader> > multiPass;

        
public:
    
    static Shader* shaderInUse;
    
    //variables
    GLint program;   //Shader program
    GLuint f, v;     //fragment and vertex shader
    
    //Uniform locations
    GLint projMatrixLoc, viewMatrixLoc, normalMatrixLoc;
    GLint nearFrustumLoc, farFrustumLoc, topFrustumLoc, bottomFrustumLoc, leftFrustumLoc,rightFrustumLoc;
    GLint hViewportLoc, wViewportLoc;
    GLint radiusSplatLoc;
    GLint colorEnabledLoc;
    GLint automaticRadiusEnabledLoc;
    GLint renderTextureLoc;
    GLint blendTextureLoc;
    GLint normalTextureLoc;
    GLint positionTextureLoc;
    GLint inverseTextureSizeLoc;
    GLint lightCountLoc;
    GLint lightPositionLoc;
    GLint lightColorLoc;
    GLint lightIntensityLoc;
    
    //Constructor
    Shader(string description, string vertexShaderPath, string fragmentShaderPath, enum shaderMode mode);
    Shader(string description, string vertexShaderPath, string fragmentShaderPath, enum shaderMode mode, vector<Shader> &multiPass);
    Shader(string description, string vertexShaderPath, string fragmentShaderPath, enum shaderMode mode, vector< vector<Shader> > &multiPass);
    
    //Getters & Setters
    void addMultiPass(vector<Shader> &multiPassVector) {multiPass.push_back(multiPassVector);};
    string getDescription() { return description; };
    string getDescription(int i) { return multiPass[i][0].description;};
    vector<Shader> &getMultiPass(int i) { return multiPass[i]; };
    vector< vector<Shader> > &getMultiPass() { return multiPass; };
    shaderMode getMode() {return mode; };
    
    void printShaderInfoLog(GLint shader);
    void bindShader();
    void compileShader();
        
};

#endif
