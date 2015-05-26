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
    vector<Shader> multiPass;
        
public:
    //variables
    static GLint program;   //Shader program
    static GLuint f, v;     //fragment and vertex shader
    
    //Uniform locations
    static GLint projMatrixLoc, viewMatrixLoc, normalMatrixLoc;
    static GLint nearFrustumLoc, farFrustumLoc, topFrustumLoc, bottomFrustumLoc, leftFrustumLoc,rightFrustumLoc;
    static GLint hViewportLoc, wViewportLoc;
    static GLint radiusSplatLoc;
    
    //Methods ...
    
    //Constructor
    Shader(string description, string vertexShaderPath, string fragmentShaderPath, enum shaderMode mode);
    Shader(string description, string vertexShaderPath, string fragmentShaderPath, enum shaderMode mode, vector<Shader> multiPass);
    
    //Getters & Setters
    void setMultiPass(vector<Shader> multiPassVector) {multiPass = multiPassVector;};
    string getDescription() { return description; };
    vector<Shader> getMultiPass() { return multiPass; };
    shaderMode getMode() {return mode; };
    
    static void printShaderInfoLog(GLint shader);
    static void bindShader();
    void compileShader();
        
};

#endif
