#include "shader.h"


#include "file.h"
#include "globals.h"

GLint   Shader::program;
GLuint  Shader::f;
GLuint  Shader::v; //fragment and vertex shader


GLint   Shader::projMatrixLoc;
GLint   Shader::viewMatrixLoc;
GLint   Shader::normalMatrixLoc;
GLint   Shader::nearFrustumLoc;
GLint   Shader::farFrustumLoc;
GLint   Shader::topFrustumLoc;
GLint   Shader::bottomFrustumLoc;
GLint   Shader::leftFrustumLoc;
GLint   Shader::rightFrustumLoc;
GLint   Shader::hViewportLoc;
GLint   Shader::wViewportLoc;

//Splat's radii
GLint   Shader::radiusSplatLoc;

//Texture
GLint   Shader::textureLoc;

//
GLint   Shader::inverseTextureSizeLoc;


Shader::Shader(string description, string vertexShaderPath, string fragmentShaderPath, shaderMode mode)
{
    this->description = description;
    this->vertexShaderPath = vertexShaderPath;
    this->fragmentShaderPath = fragmentShaderPath;
    this->mode = mode;
}


Shader::Shader(string description, string vertexShaderPath, string fragmentShaderPath, enum shaderMode mode, vector<Shader> multiPass)
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
    Shader::program = glCreateProgram();
    
    glAttachShader(Shader::program, Shader::v);
    glAttachShader(Shader::program, Shader::f);
    
    glBindAttribLocation(Shader::program, 0, "in_Position");
    glBindAttribLocation(Shader::program, 1, "in_Color");
    glBindAttribLocation(Shader::program, 2, "in_Normals");
    
    glLinkProgram(Shader::program);
    
    glDeleteShader(Shader::v);
    glDeleteShader(Shader::f);
    
    glUseProgram(Shader::program);
    glDeleteProgram(Shader::program);
    
    Shader::projMatrixLoc = glGetUniformLocation(Shader::program, "projMatrix");
    Shader::viewMatrixLoc = glGetUniformLocation(Shader::program, "viewMatrix");
    Shader::normalMatrixLoc = glGetUniformLocation(Shader::program, "normalMatrix");
    Shader::nearFrustumLoc = glGetUniformLocation(Shader::program, "n");
    Shader::farFrustumLoc = glGetUniformLocation(Shader::program, "f");
    Shader::topFrustumLoc = glGetUniformLocation(Shader::program,"t");
    Shader::bottomFrustumLoc = glGetUniformLocation(Shader::program,"b");
    Shader::rightFrustumLoc = glGetUniformLocation(Shader::program,"r");
    Shader::leftFrustumLoc = glGetUniformLocation(Shader::program, "l");
    Shader::hViewportLoc = glGetUniformLocation(Shader::program,"h");
    Shader::wViewportLoc = glGetUniformLocation(Shader::program,"w");
    Shader::radiusSplatLoc = glGetUniformLocation(Shader::program,"radius");
    
    Shader::textureLoc = glGetUniformLocation(Shader::program, "myTexture");
    
    Shader::inverseTextureSizeLoc = glGetUniformLocation(Shader::program, "inverseTextureSize");
    
    
    glUniform1f(Shader::radiusSplatLoc, radiusSplat);
}



void Shader::compileShader()
{
    
    char *vs,*fs;
    
    Shader::v = glCreateShader(GL_VERTEX_SHADER);
    Shader::f = glCreateShader(GL_FRAGMENT_SHADER);
    
    // load shaders & get length of each
    GLint vlen;
    GLint flen;
    
    vs = loadFile( PATH_TO_SHADERS + vertexShaderPath, vlen);
    fs = loadFile( PATH_TO_SHADERS + fragmentShaderPath, flen);
    
    const char * vv = vs;
    const char * ff = fs;
    
    glShaderSource(Shader::v, 1, &vv, &vlen);
    glShaderSource(Shader::f, 1, &ff, &flen);
    
    GLint compiled;
    
    glCompileShader(Shader::v);
    glGetShaderiv(v, GL_COMPILE_STATUS, &compiled);
    if (!compiled)
    {
        cout << "Vertex shader not compiled." << endl;
        printShaderInfoLog(Shader::v);
    }
    
    glCompileShader(Shader::f);
    glGetShaderiv(Shader::f, GL_COMPILE_STATUS, &compiled);
    if (!compiled)
    {
        cout << "Fragment shader not compiled." << endl;
        printShaderInfoLog(Shader::f);
    }
    
    delete [] vs; // dont forget to free allocated memory
    delete [] fs; // we allocated this in the loadFile function...
}

