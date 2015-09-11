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

#ifdef _MSC_VER
#include <io.h>
#else
#include <unistd.h>
#endif
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include <ctime>

#include <GL/glew.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <GLFW/glfw3.h>

#include "globals.h"
#include "file.h"
#include "vao.h"
#include "shader.h"
#include "orbitallight.h"
#include "camera.h"

#define DEBUG
#define ITERATIONS 25

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

#define CAMERA_RPP 360.0/1000.0 //resolution 1000px = 2PI

#define SGN(x)   (((x) < 0) ? (-1) : (1))
#define LESS_THAN(x, limit) ((x > limit) ? (limit) : (x))
#define GREATER_THAN(x, limit) ((x < limit) ? (limit) : (x))


using namespace std;

#ifdef DEBUG
#include <fstream>
std::chrono::time_point<std::chrono::system_clock> startTime, endTime;
int itCounter = 0;
int numOfLights = 0;
int numOfPoints = 0;
ofstream logStream;
#endif

GLuint FramebufferName = 0;
GLuint fbufferTex[4];
GLuint depthrenderbuffer;

/**
 @brief Returns a title for the window
 Concatenate 'CUBE' with the description of the shader thats its been used
 @returns title
 */
const char* getTitleWindow()
{

    string multipass;
    if (Globals::MultipassEnabled){
        unsigned int indexMultipass = Globals::actualMultipass % Globals::listOfShaders[Globals::actualShader%Globals::listOfShaders.size()].getMultiPass().size();
        multipass = Globals::listOfShaders[Globals::actualShader%Globals::listOfShaders.size()].getDescription(indexMultipass) + " Shading";
    }
    else
        multipass = "Flat Shading";
    
    string color;
    if (Globals::colorEnabled)
        color = " | NONE ";
    else
        color = " | RGB ";
    
    string fxaa;
    if (Globals::FXAA)
        fxaa = " | FXAA";
    else
        fxaa = "";
    
    Globals::title = "CUBE | " + Globals::listOfShaders[Globals::actualShader%Globals::listOfShaders.size()].getDescription() + " | " + multipass + color + fxaa;
    return Globals::title.c_str();
}

#ifdef DEBUG
    void writeTitleLog(){
        numOfLights = Globals::sceneLightsList[ Globals::sceneLightsArrIndex % Globals::sceneLightsList.size()].size();
        numOfPoints = Globals::Globals::displayVAO->getCloud()->points.size();
        itCounter = 0;
        logStream << getTitleWindow() << "| " << numOfPoints << " Points | " << numOfLights << " Lights" << endl;
    }
#endif


/**
 Sample a model transforming it into a point cloud
 @param mesh model defined by GL_TRIANGLES
 @returns point cloud
 */



void drawWindowSizedRectangle()
{
    float points[] = {
        -1.0f, -1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        1.0f,  1.0f, 0.0f,
    };
    
    GLuint vbo = 0;
    glGenBuffers (1, &vbo);
    glBindBuffer (GL_ARRAY_BUFFER, vbo);
    glBufferData (GL_ARRAY_BUFFER, sizeof(points), points, GL_STATIC_DRAW);
    
    GLuint vao = 0;
    glGenVertexArrays (1, &vao);
    glBindVertexArray (vao);
    glEnableVertexAttribArray (0);
    glBindBuffer (GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    // draw points 0-3 from the currently bound VAO with current in-use shader
    glDrawArrays (GL_TRIANGLES, 0, 6);
    
    glBindVertexArray(0);

}




void reshapeCallback(GLFWwindow * window, int w, int h)
{
    // set viewport to be the entire window
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    
    // resize framebuffer
    glBindTexture(GL_TEXTURE_RECTANGLE, fbufferTex[0]);
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGB8, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
    glBindTexture(GL_TEXTURE_RECTANGLE, fbufferTex[1]);
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGBA16F, w, h, 0, GL_RGBA, GL_FLOAT, 0);
    glBindTexture(GL_TEXTURE_RECTANGLE, fbufferTex[2]);
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGBA16F, w, h, 0, GL_RGBA, GL_FLOAT, 0);
    glBindTexture(GL_TEXTURE_RECTANGLE, fbufferTex[3]);
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGB32F, w, h, 0, GL_RGB, GL_FLOAT, 0);
    
    glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);
    
    if (Camera::activeCamera != NULL)
        Camera::activeCamera->update(w, h);
    
    Globals::firstTime = true;
}


void updateLightPosition()
{
    vector<Light*> lightList = Globals::sceneLightsList[ Globals::sceneLightsArrIndex % Globals::sceneLightsList.size()];
    
    for (int i =0; i < lightList.size(); i++)
        lightList[i]->update();
    
    Light::pushToGPU(lightList);
}


void scrollCallback(GLFWwindow * window, double xoffset, double yoffset)
{
    
    
    int w, h;
    glfwGetWindowSize(window, &w, &h);
    
    if (Camera::activeCamera != NULL) {
        Camera::activeCamera->moveDistance( - yoffset );
        Camera::activeCamera->update(w,h);
    }
}



void mousePosCallback(GLFWwindow * window, double x, double y)
{
    if (Globals::leftBtnPress == true) {
        int w, h;
        glfwGetWindowSize(window, &w, &h);
        if (Camera::activeCamera != NULL) {
            Camera::activeCamera->rotate((Globals::lastMouseX - x) * CAMERA_RPP, -(Globals::lastMouseY - y) * CAMERA_RPP);
            Camera::activeCamera->update(w, h);
        }
    }
    
    Globals::lastMouseX = x;
    Globals::lastMouseY = y;

}



void mouseCallback(GLFWwindow * window, int btn, int action, int mods)
{
	if(btn == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_RELEASE)
            Globals::leftBtnPress = false;
        else if (action == GLFW_PRESS)
            Globals::leftBtnPress = true;
    }
}



void keyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
    
    if (key == GLFW_KEY_UP) {
        Globals::userRadiusFactor += 0.001f;
        glUniform1f(Shader::shaderInUse->radiusSplatLoc, Globals::userRadiusFactor);
    }
    
    if (key == GLFW_KEY_DOWN) {
        Globals::userRadiusFactor -= 0.001f;
        glUniform1f(Shader::shaderInUse->radiusSplatLoc, Globals::userRadiusFactor);
    }
    
    if (key == GLFW_KEY_A && action == GLFW_PRESS) {
        Globals::automaticRadiusEnabled = !Globals::automaticRadiusEnabled;
        
        if (Globals::automaticRadiusEnabled) {
            Globals::backupUserRadiusFactor = Globals::userRadiusFactor;
            Globals::userRadiusFactor = 1.0f;
        }
        else
            Globals::userRadiusFactor = Globals::backupUserRadiusFactor;
        
        glUniform1f(Shader::shaderInUse->radiusSplatLoc, Globals::userRadiusFactor);
        glUniform1i(Shader::shaderInUse->automaticRadiusEnabledLoc, Globals::automaticRadiusEnabled?1:0);
    }
    
    if (key == GLFW_KEY_C && action == GLFW_PRESS) {
        Globals::colorEnabled = !Globals::colorEnabled;
        glUniform1i(Shader::shaderInUse->colorEnabledLoc, Globals::colorEnabled?1:0);
        glfwSetWindowTitle(window, getTitleWindow());
        
        #ifdef DEBUG
        writeTitleLog();
        #endif
        
    }
    
    if (key == GLFW_KEY_F && action == GLFW_PRESS) {
        Globals::FXAA = !Globals::FXAA;
        
        glfwSetWindowTitle(window, getTitleWindow());
        
        #ifdef DEBUG
        writeTitleLog();
        #endif
        
    }
    
    if (key == GLFW_KEY_L && action == GLFW_PRESS) {
        Globals::sceneLightsArrIndex += 1;
        
        #ifdef DEBUG
        writeTitleLog();
        #endif
        
    }
    
    if (key == GLFW_KEY_M && action == GLFW_PRESS) {
        Globals::actualVAO++;
        Globals::displayVAO = &Globals::models[Globals::actualVAO%Globals::models.size()];
        
        #ifdef DEBUG
        writeTitleLog();
        #endif
    }
    
    if (key == GLFW_KEY_O && action == GLFW_PRESS) {
        string pathToFile;
        cout << "Open File: ";
        cin >> pathToFile;
        VAO VAO = loadCloud(pathToFile);
        if (VAO.isValid() ) {
            Globals::models.push_back(VAO);
            Globals::actualVAO = Globals::models.size() - 1;
            Globals::displayVAO = &Globals::models[Globals::models.size()-1];
            Globals::displayVAO->pushToGPU();
        }
        
        #ifdef DEBUG
        writeTitleLog();
        #endif
        
    }
    
    if (key == GLFW_KEY_P && action == GLFW_PRESS) {
        
        int indexShader = Globals::actualShader % Globals::listOfShaders.size();
        
        if (Globals::MultipassEnabled) {
            Globals::actualMultipass++;
            
            int indexMultiPass = Globals::actualMultipass % Globals::listOfShaders[indexShader].getMultiPass().size();
            
            if (indexMultiPass == 0)
                Globals::MultipassEnabled = false;
        }
        else
            if (Globals::listOfShaders[indexShader].getMultiPass().size() > 0 )
                Globals::MultipassEnabled = true;
        
        glfwSetWindowTitle(window, getTitleWindow());
        
        #ifdef DEBUG
        writeTitleLog();
        #endif
        
    }
    
    if (key == GLFW_KEY_Q && action == GLFW_PRESS) {
        Globals::listOfShaders[Globals::actualShader%Globals::listOfShaders.size()].compileShader();
    }
    
    if (key == GLFW_KEY_R && action == GLFW_PRESS) {
        int w, h;
        glfwGetWindowSize(window, &w, &h);
        if (Camera::activeCamera != NULL) {
            Camera::activeCamera->reset();
            Camera::activeCamera->update(w, h);
        }
    }
    
    if (key == GLFW_KEY_S && action == GLFW_PRESS) {
        Globals::actualShader++;
        
        if (Globals::listOfShaders[Globals::actualShader%Globals::listOfShaders.size()].getMultiPass().empty()) {
            Globals::MultipassEnabled = false;
            Globals::actualMultipass = 0;
        }
        
        #ifdef DEBUG
        writeTitleLog();
        #endif
        
        glfwSetWindowTitle(window, getTitleWindow());
    }
    
}



void applyFXAA(GLFWwindow * window)
{
    
    int windowWidth, windowHeight;
    glfwGetWindowSize(window, &windowWidth, &windowHeight);
    
    //fxaaFilter.compileShader();
    Globals::fxaaFilter->bindShader();
    
    //Copy framebuffer to a Texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_RECTANGLE, Globals::textureID);
    glEnable(GL_TEXTURE_RECTANGLE);
    if (Globals::firstTime){
        glCopyTexImage2D(GL_TEXTURE_RECTANGLE,0, GL_RGBA16F, 0, 0, windowWidth, windowHeight, 0);
        Globals::firstTime = false;
    }
    else
        glCopyTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, 0, 0, windowWidth, windowHeight);
    
    glUniform1i(Shader::shaderInUse->renderTextureLoc, 0);
    glUniform3f(Shader::shaderInUse->inverseTextureSizeLoc, 1.0f/windowWidth, 1.0f/windowHeight, 0.0f);
    
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawWindowSizedRectangle();
    
    glBindVertexArray(0);

}



void display(GLFWwindow* window)
{
    
#ifdef DEBUG
    
    if (itCounter == 0)
        startTime = std::chrono::system_clock::now();

    itCounter++;

#endif
    
    int windowWidth, windowHeight;
    glfwGetWindowSize(window, &windowWidth, &windowHeight);
    
    if (Globals::displayVAO != NULL) {
    glBindVertexArray(Globals::displayVAO->getVAOid());
        
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glViewport(0, 0, windowWidth, windowHeight);

    if (Globals::displayVAO != NULL) {
        
        if (!Globals::MultipassEnabled) {
            Globals::listOfShaders[Globals::actualShader%Globals::listOfShaders.size()].bindShader();
            
            glClearColor(86.f/255.f,136.f/255.f,199.f/255.f,1.0f);
            glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glDepthFunc(GL_LEQUAL);
            glDrawArrays(Globals::displayVAO->getMode(), 0, Globals::displayVAO->getCloud()->points.size());
        }
        else {
            glDepthMask(GL_TRUE);
            glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
            Shader shaderh = Globals::listOfShaders[Globals::actualShader%Globals::listOfShaders.size()];
            unsigned int indexMultipass = Globals::actualMultipass % shaderh.getMultiPass().size();
            
            for (unsigned int i = 0; i < shaderh.getMultiPass(indexMultipass).size(); i++) {
                
                shaderh.getMultiPass(indexMultipass)[i].bindShader();
                
                switch (shaderh.getMultiPass(indexMultipass)[i].getMode()) {
                        
                    case shader::DEPTH_MASK:
                    {
                        glDepthMask(GL_TRUE);
                        GLenum attach[2] = {GL_NONE, GL_COLOR_ATTACHMENT3};
                        glDrawBuffers(2, attach);
                        glDrawArrays(Globals::displayVAO->getMode(), 0, Globals::displayVAO->getCloud()->points.size());
                        break;
                    }
                    case shader::BLENDING:
                    {
                        glEnable(GL_BLEND);
                        glDepthMask(GL_FALSE);
                        glDepthFunc(GL_LEQUAL);
                        GLenum attach[2] = {GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
                        glDrawBuffers(2, attach);
                        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
                        glClear(GL_COLOR_BUFFER_BIT);
                        glDrawArrays(Globals::displayVAO->getMode(), 0, Globals::displayVAO->getCloud()->points.size());
                        break;
                    }
                    case shader::NORMALIZATION:
                    {
                        glActiveTexture(GL_TEXTURE0);
                        glBindTexture(GL_TEXTURE_RECTANGLE, fbufferTex[1]);
                        glUniform1i(Shader::shaderInUse->blendTextureLoc, 0);
                        
                        glActiveTexture(GL_TEXTURE1);
                        glBindTexture(GL_TEXTURE_RECTANGLE, fbufferTex[2]);
                        glUniform1i(Shader::shaderInUse->normalTextureLoc, 1);
                        
                        glActiveTexture(GL_TEXTURE2);
                        glBindTexture(GL_TEXTURE_RECTANGLE, fbufferTex[3]);
                        glUniform1i(Shader::shaderInUse->positionTextureLoc, 2);
                        
                        //Render Texture and normalize
                        glDrawBuffer(GL_COLOR_ATTACHMENT0);
                        glClearColor(86.f/255.f,136.f/255.f,199.f/255.f,1.0f);
                        glClear(GL_COLOR_BUFFER_BIT);
                        drawWindowSizedRectangle();
                        break;
                    }
                    default:
                        break;
                }
                
                glDisable(GL_BLEND);
                glDepthMask(GL_TRUE);
            }
            
        }
    }
    
    glBindVertexArray(0);
    
    if (Globals::FXAA)
        applyFXAA(window);
    
    //Blit framebuffer resultant to window
    glBindFramebuffer(GL_READ_FRAMEBUFFER, FramebufferName);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, windowWidth, windowHeight,
                      0, 0, windowWidth, windowHeight, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        
    }
    
#ifdef DEBUG
    
    if (itCounter >= ITERATIONS) {
        endTime = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = endTime-startTime;
        logStream << elapsed_seconds.count()/ITERATIONS << endl;
        itCounter = 0;
    }


#endif
    
}

void buildFBO()
{
    // ---------------------------------------------
    // Render to Texture - specific code begins here
    // ---------------------------------------------
    
    // The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
    glGenFramebuffers(1, &FramebufferName);
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
    
    // The texture we're going to render to
    // The texture we're going to render to
    glGenTextures(4, fbufferTex);
    
    // "Bind" the newly created texture : all future texture functions will modify this texture
    glBindTexture(GL_TEXTURE_RECTANGLE, fbufferTex[0]);
    
    // Give an empty image to OpenGL ( the last "0" means "empty" )
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGB8, WINDOW_WIDTH, WINDOW_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
    
    // Poor filtering
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    
    // "Bind" the newly created texture : all future texture functions will modify this texture
    glBindTexture(GL_TEXTURE_RECTANGLE, fbufferTex[1]);
    
    // Give an empty image to OpenGL ( the last "0" means "empty" )
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGBA16F, WINDOW_WIDTH, WINDOW_HEIGHT, 0, GL_RGBA, GL_FLOAT, 0);
    
    // Poor filtering
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    
    // "Bind" the newly created texture : all future texture functions will modify this texture
    glBindTexture(GL_TEXTURE_RECTANGLE, fbufferTex[2]);
    // Give an empty image to OpenGL ( the last "0" means "empty" )
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGBA16F, WINDOW_WIDTH, WINDOW_HEIGHT, 0, GL_RGBA, GL_FLOAT, 0);
    // Poor filtering
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    
    //// Alternative : Depth texture. Slower, but you can sample it later in your shader
    glBindTexture(GL_TEXTURE_RECTANGLE, fbufferTex[3]);
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGB32F, WINDOW_WIDTH, WINDOW_HEIGHT, 0, GL_RGB, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // The depth buffer
    glGenRenderbuffers(1, &depthrenderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, WINDOW_WIDTH, WINDOW_HEIGHT);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthrenderbuffer);
    
    // Set "renderedTexture" as our colour attachement #0
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, fbufferTex[0], 0);
    
    
    // Set "blendTexture" as our colour attachement #0
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, fbufferTex[1], 0);
    
    // Set "normalsTexture" as our colour attachement #0
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, fbufferTex[2], 0);
    
    //// Depth texture alternative :
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, fbufferTex[3], 0);
    
    
    // Set the list of draw buffers.
    
    // Always check that our framebuffer is ok
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        exit(1);
}


int main(int argc, char **argv)
{
#ifdef DEBUG
    logStream.open ("log.txt");
#endif
    
    Globals::init();
    
    GLFWwindow* window;
    
    /* Initialize the library */
    if (!glfwInit())
        return -1;
    
    /* create context */
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "CUBE", NULL, NULL);
    glfwSetWindowTitle(window, getTitleWindow());
    
    if (!window)
    {
        glfwTerminate();
        return -1;
    }
    
    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    //Glew Init
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        cout << "glewInit failed, aborting." << endl;
        exit (1);
    }
    
    if (!GLEW_EXT_framebuffer_object)
    {
        printf("Error: no extension GL_EXT_framebuffer_object.");
        return 0;
    }
    
    if (!GLEW_ARB_color_buffer_float)
    {
        printf("Error: no extension ARB_color_buffer_float.");
        return 0;
    }
    
    glEnable(GL_TEXTURE_RECTANGLE);
    
    //FrameBuffer for rendering in multipass mode
    buildFBO();
    
    //Texture used for normalize
    glGenTextures(1, &Globals::textureID);
    
    glClampColor(GL_CLAMP_READ_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_VERTEX_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_FRAGMENT_COLOR, GL_FALSE);
    
    
    /*openGL configure*/
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointParameteri(GL_POINT_SPRITE_COORD_ORIGIN, GL_LOWER_LEFT);
    glBlendFuncSeparateEXT(GL_SRC_ALPHA, GL_ONE, GL_ONE, GL_ONE);
    
    cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
    cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
    cout << "GLEW version: " << glewGetString(GLEW_VERSION) << endl;
    
    GLint maxColorAttachments;
    glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS, &maxColorAttachments);
    
    cout << "GL_MAX_COLOR_ATTACHMENTS: " << maxColorAttachments << endl;
    
    GLint maxTextureImageUnits;
    glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &maxTextureImageUnits);
    
    cout << "GL_MAX_TEXTURE_IMAGE_UNITS: " << maxTextureImageUnits << endl;
    
    //init all models
    int w, h;
    glfwGetWindowSize(window, &w, &h);
    
    if (Camera::activeCamera != NULL)
        Camera::activeCamera->update(w, h);
    
    cubeMesh.sampleMesh(500);
    Globals::models.push_back(cubeMesh);
    VAO sphere;
    sphere.sampleSphere(2000);
    Globals::models.push_back(sphere);
    for (unsigned int i = 0; i < Globals::models.size(); i++) {
        Globals::models[i].pushToGPU();
    }
    
    Globals::displayVAO = &Globals::models[0];
    
    
    Globals::fxaaFilter->compileShader();
    
    for (unsigned int i = 0; i < Globals::listOfShaders.size(); i ++) {
        Globals::listOfShaders[i].compileShader();
        
        if ( !Globals::listOfShaders[i].getMultiPass().empty()) {
            for (unsigned int ii = 0; ii < Globals::listOfShaders[i].getMultiPass().size(); ii ++)
                for (unsigned int iii = 0; iii < Globals::listOfShaders[i].getMultiPass(ii).size(); iii ++)
                    Globals::listOfShaders[i].getMultiPass(ii)[iii].compileShader();
        }
        
    }
    
    Globals::listOfShaders[Globals::actualShader].bindShader();
    
    /*glfw Callbacks*/
    glfwSetKeyCallback(window, keyboardCallback);
    glfwSetWindowSizeCallback(window, reshapeCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetMouseButtonCallback(window, mouseCallback);
    glfwSetCursorPosCallback(window, mousePosCallback);
    reshapeCallback(window, WINDOW_WIDTH, WINDOW_HEIGHT); //callback forced
    
    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        updateLightPosition();
        
        /* Render here */
        display(window);
        
        /* Swap front and back buffers */
        glfwSwapBuffers(window);
        
        /* Poll for and process events */
        glfwPollEvents();
    }
#ifdef DEBUG
    logStream.close();
#endif

    glfwTerminate();
    return 0;

}
