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

#include "shader.h"


#include "file.h"
#include "globals.h"

#include "orbitallight.h"
#include "camera.h"

Shader* Shader::shaderInUse = NULL;


Shader::Shader(string description, string vertexShaderPath, string fragmentShaderPath, shaderMode mode)
{
    this->description = description;
    this->vertexShaderPath = vertexShaderPath;
    this->fragmentShaderPath = fragmentShaderPath;
    this->mode = mode;
}


Shader::Shader(string description, string vertexShaderPath, string fragmentShaderPath, enum shaderMode mode, vector<Shader> &multiPass)
{
    this->description = description;
    this->vertexShaderPath = vertexShaderPath;
    this->fragmentShaderPath = fragmentShaderPath;
    this->mode = mode;
    this->multiPass.push_back( multiPass);
}

Shader::Shader(string description, string vertexShaderPath,
               string fragmentShaderPath,
               enum shaderMode mode,
               vector< vector<Shader> > &multiPass)
{
    this->description = description;
    this->vertexShaderPath = vertexShaderPath;
    this->fragmentShaderPath = fragmentShaderPath;
    this->mode = mode;
    this->multiPass = multiPass;
}

/**
 @brief Display (hopefully) useful error messages if shader fails to compile
 From OpenGL Shading Language 3rd Edition, p215-216
 @param shader shader reference
 @returns void
 */
void Shader::printShaderInfoLog(GLint shader)
{
    int infoLogLen = 0;
    int charsWritten = 0;
    GLchar *infoLog;
    
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLen);
    
    // should additionally check for OpenGL errors here
    
    if (infoLogLen > 0)
    {
        infoLog = new GLchar[infoLogLen];
        // error check for fail to allocate memory omitted
        glGetShaderInfoLog(shader,infoLogLen, &charsWritten, infoLog);
        cout << "InfoLog:" << endl << infoLog << endl;
        delete [] infoLog;
    }
    
    // should additionally check for OpenGL errors here
}



void Shader::bindShader()
{
    program = glCreateProgram();
    
    glAttachShader(program, v);
    glAttachShader(program, f);
    
    glBindAttribLocation(program, 0, "in_Position");
    glBindAttribLocation(program, 1, "in_Color");
    glBindAttribLocation(program, 2, "in_Normals");
    glBindAttribLocation(program, 3, "in_Radius");
    
    glLinkProgram(program);
    
    //glDeleteShader(v);
    //glDeleteShader(f);
    
    glUseProgram(program);
    glDeleteProgram(program);
    
    Shader::shaderInUse = this;
    
    projMatrixLoc = glGetUniformLocation(program, "projMatrix");
    viewMatrixLoc = glGetUniformLocation(program, "viewMatrix");
    normalMatrixLoc = glGetUniformLocation(program, "normalMatrix");
    nearFrustumLoc = glGetUniformLocation(program, "n");
    farFrustumLoc = glGetUniformLocation(program, "f");
    topFrustumLoc = glGetUniformLocation(program,"t");
    bottomFrustumLoc = glGetUniformLocation(program,"b");
    rightFrustumLoc = glGetUniformLocation(program,"r");
    leftFrustumLoc = glGetUniformLocation(program, "l");
    hViewportLoc = glGetUniformLocation(program,"h");
    wViewportLoc = glGetUniformLocation(program,"w");
    radiusSplatLoc = glGetUniformLocation(program, "userRadiusFactor");
    
    lightPositionLoc = glGetUniformLocation(program, "lightPosition");
    lightColorLoc = glGetUniformLocation(program, "lightColor");
    lightIntensityLoc =  glGetUniformLocation(program, "lightIntensity");
    lightCountLoc = glGetUniformLocation(program, "lightCount");
    
    renderTextureLoc = glGetUniformLocation(program, "renderTexture");
    blendTextureLoc = glGetUniformLocation(program, "blendTexture");
    normalTextureLoc = glGetUniformLocation(program, "normalTexture");
    positionTextureLoc = glGetUniformLocation(program, "positionTexture");
    
    inverseTextureSizeLoc = glGetUniformLocation(program, "inverseTextureSize");
    colorEnabledLoc = glGetUniformLocation(program, "colorEnabled");
    automaticRadiusEnabledLoc = glGetUniformLocation(program, "automaticRadiusEnabled");
    
    //Lights
    glUniform3fv(lightPositionLoc, MAX_LIGHTS , OrbitalLight::lightPosition );
    glUniform3fv(lightColorLoc, MAX_LIGHTS , OrbitalLight::lightColor );
    glUniform1fv(lightIntensityLoc, MAX_LIGHTS, OrbitalLight::lightIntensity);
    glUniform1iv(lightCountLoc, 1, &OrbitalLight::lightCount);
    
    //Camera
    glUniformMatrix4fv(viewMatrixLoc,  1, false, glm::value_ptr(Camera::viewMatrix));
    glUniformMatrix3fv(normalMatrixLoc, 1, false, glm::value_ptr(Camera::normalMatrix));
    glUniformMatrix4fv(projMatrixLoc,  1, false, glm::value_ptr(Camera::projMatrix));
    glUniform1iv(hViewportLoc, 1, &Camera::h);
    glUniform1iv(wViewportLoc, 1, &Camera::w);
    glUniform1fv(nearFrustumLoc, 1, &Camera::n);
    glUniform1fv(farFrustumLoc, 1, &Camera::f);
    glUniform1fv(topFrustumLoc, 1, &Camera::top);
    glUniform1fv(bottomFrustumLoc, 1, &Camera::bottom);
    glUniform1fv(leftFrustumLoc, 1, &Camera::left);
    glUniform1fv(rightFrustumLoc, 1, &Camera::right);
    
    glUniform1f(automaticRadiusEnabledLoc, Globals::automaticRadiusEnabled);
    glUniform1f(colorEnabledLoc, Globals::colorEnabled);
    glUniform1f(radiusSplatLoc, Globals::userRadiusFactor);
}



void Shader::compileShader()
{
    
    char *vs,*fs;
    
    v = glCreateShader(GL_VERTEX_SHADER);
    f = glCreateShader(GL_FRAGMENT_SHADER);
    
    // load shaders & get length of each
    GLint vlen;
    GLint flen;
    
    vs = loadFile( PATH_TO_SHADERS + vertexShaderPath, vlen);
    fs = loadFile( PATH_TO_SHADERS + fragmentShaderPath, flen);
    
    const char * vv = vs;
    const char * ff = fs;
    
    glShaderSource(v, 1, &vv, &vlen);
    glShaderSource(f, 1, &ff, &flen);
    
    GLint compiled;
    
    glCompileShader(v);
    glGetShaderiv(v, GL_COMPILE_STATUS, &compiled);
    if (!compiled)
    {
        cout << "Vertex shader not compiled." << endl;
        printShaderInfoLog(v);
    }
    
    glCompileShader(f);
    glGetShaderiv(f, GL_COMPILE_STATUS, &compiled);
    if (!compiled)
    {
        cout << "Fragment shader not compiled." << endl;
        printShaderInfoLog(f);
    }
    
    delete [] vs; // dont forget to free allocated memory
    delete [] fs; // we allocated this in the loadFile function...
}

