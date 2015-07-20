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
    GLint textureLoc;
    GLint inverseTextureSizeLoc;
    
    //Methods ...
    
    //Constructor
    Shader(string description, string vertexShaderPath, string fragmentShaderPath, enum shaderMode mode);
    Shader(string description, string vertexShaderPath, string fragmentShaderPath, enum shaderMode mode, vector<Shader> &multiPass);
    
    //Getters & Setters
    void setMultiPass(vector<Shader> &multiPassVector) {multiPass = multiPassVector;};
    string getDescription() { return description; };
    vector<Shader> &getMultiPass() { return multiPass; };
    shaderMode getMode() {return mode; };
    
    void printShaderInfoLog(GLint shader);
    void bindShader();
    void compileShader();
        
};

#endif
