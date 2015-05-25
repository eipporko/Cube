#ifndef CUBE_shader_h
#define CUBE_shader_h

#include <iostream>
#include <vector>

#include <GL/glew.h>

#define PATH_TO_SHADERS "../src/shaders/" 


using namespace std;


enum shaderMode {
    SINGLEPASS      = 0,
    DEPTH_MASK      = 1,
    BLENDING        = 2,
    NORMALIZATION   = 3
};


class Shader
{
private:
    string description;
    string vertexShaderPath;
    string fragmentShaderPath;
    shaderMode mode;
    vector<Shader> multiPass;
    
public:
    static void printShaderInfoLog(GLint shader);
    
    void compileShader();
    
};

#endif
