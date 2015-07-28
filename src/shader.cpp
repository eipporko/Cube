#include "shader.h"


#include "file.h"
#include "globals.h"


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
    
    textureLoc = glGetUniformLocation(program, "myTexture");
    
    inverseTextureSizeLoc = glGetUniformLocation(program, "inverseTextureSize");
    colorEnabledLoc = glGetUniformLocation(program, "colorEnabled");
    automaticRadiusEnabledLoc = glGetUniformLocation(program, "automaticRadiusEnabled");
    
    glUniform1f(automaticRadiusEnabledLoc, automaticRadiusEnabled);
    glUniform1f(colorEnabledLoc, colorEnabled);
    glUniform1f(radiusSplatLoc, userRadiusFactor);
    
    Shader::shaderInUse = this;
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

