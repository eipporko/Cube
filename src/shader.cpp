#include "shader.h"


#include "file.h"
#include "globals.h"

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

