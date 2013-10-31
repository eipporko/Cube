#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <GL/glew.h>

#include "glm/glm.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/rotate_vector.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#elif
#include <GL/glut.h>
#endif

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

#define CAMERA_DISTANCE 7.0
#define CAMERA_DPP 360.0/1000.0 //360º = 1000px

#define SGN(x)   (((x)<0) ? (-1) : (1))


using namespace std;



GLuint vao, vbo[2];
GLint projMatrixLoc, viewMatrixLoc; //uniform locations
glm::mat4 projMatrix, viewMatrix; //transformation matrix

GLint program; //shader program

glm::vec3 cameraEye = glm::vec3(0, 0, -CAMERA_DISTANCE);
glm::vec3 cameraUp = glm::vec3(0,1,0);

int lastMouseX = NULL, lastMouseY = NULL; //last mouse position pressed;


//Cube vertices and color description
GLfloat vertices[]  = { -1, 1, 1,  -1, 1,-1,   1, 1, 1,  //Top
                         1, 1, 1,  -1, 1,-1,   1, 1,-1,
    
                         1, 1,-1,  -1, 1,-1,  -1,-1,-1,  //Front
                        -1,-1,-1,   1,-1,-1,   1, 1,-1,
    
                         1, 1,-1,   1,-1,-1,   1, 1, 1,  //Right
                         1, 1, 1,   1,-1,-1,   1,-1, 1,
    
                         1,-1, 1,   1,-1,-1,  -1,-1, 1,  //Bottom
                        -1,-1, 1,   1,-1,-1,  -1,-1,-1,
    
                        -1,-1,-1,  -1, 1,-1,  -1,-1, 1,  //Left
                        -1,-1, 1,  -1, 1,-1,  -1, 1, 1,
    
                        -1, 1, 1,   1, 1 ,1,   1,-1, 1,  //Back
                         1,-1, 1,  -1,-1, 1,  -1, 1, 1 };

GLfloat colors[]    = {  0, 0, 1,   0, 0, 1,   0, 0, 1,  //Top (blue)
                         0, 0, 1,   0, 0, 1,   0, 0, 1,
    
                         0, 1, 0,   0, 1, 0,   0, 1, 0,  //Front (green)
                         0, 1, 0,   0, 1, 0,   0, 1, 0,
    
                         0, 1, 1,   0, 1, 1,   0, 1, 1,  //Right (cyan)
                         0, 1, 1,   0, 1, 1,   0, 1, 1,
    
                         1, 0, 0,   1, 0, 0,   1, 0, 0,  //Bottom (red)
                         1, 0, 0,   1, 0, 0,   1, 0, 0,
    
                         1, 0, 1,   1, 0, 1,   1, 0, 1,  //Left (magenta)
                         1, 0, 1,   1, 0, 1,   1, 0, 1,
    
                         1, 1, 0,   1, 1, 0,   1, 1, 0,  //Back (yellow)
                         1, 1, 0,   1, 1, 0,   1, 1, 0 };


char* loadFile(string fname, GLint &fSize)
{
	ifstream::pos_type size;
	char * memblock;
	string text;
    
	// file read based on example in cplusplus.com tutorial
	ifstream file (fname, ios::in|ios::binary|ios::ate);
	if (file.is_open())
	{
		size = file.tellg();
		fSize = (GLuint) size;
		memblock = new char [size];
		file.seekg (0, ios::beg);
		file.read (memblock, size);
		file.close();
		cout << "file " << fname << " loaded" << endl;
		text.assign(memblock);
	}
	else
	{
		cout << "Unable to open file " << fname << endl;
		exit(1);
	}
	return memblock;
}


// printShaderInfoLog
// From OpenGL Shading Language 3rd Edition, p215-216
// Display (hopefully) useful error messages if shader fails to compile
void printShaderInfoLog(GLint shader)
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


void initVAO()
{
    
    if (GLEW_ARB_vertex_buffer_object)
    {
        cout << "Video card supports GL_ARB_vertex_buffer_object." << endl;
        
        /* Allocate and assign a Vertex Array Object to our handle */
        glGenVertexArrays(1, &vao);
        
        /* Bind our Vertex Array Object as the current used object */
        glBindVertexArray(vao);
        
        glGenBuffers(2, vbo);
        
        /* Bind our first VBO as being the active buffer and storing vertex attributes (coordinates) */
        glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
        
        /* Copy the vertex data from vertices to our buffer */
        /* sizeof(vertices) is the size of the vertices array */
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        
        /* Specify that our coordinate data is going into attribute index 0, and contains three floats per vertex */
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        
        /* Enable attribute index 0 as being used */
        glEnableVertexAttribArray(0);
        
        /* Bind our second VBO as being the active buffer and storing vertex attributes (colors) */
        glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
        
        /* Copy the color data from colors to our buffer */
        /* sizeof(colors) is the size of the colors array */
        glBufferData(GL_ARRAY_BUFFER, sizeof(colors), colors, GL_STATIC_DRAW);
        
        /* Specify that our color data is going into attribute index 1, and contains three floats per vertex */
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
        
        /* Enable attribute index 1 as being used */
        glEnableVertexAttribArray(1);
        
    }
    else
    {
        cout << "Video card does NOT support GL_ARB_vertex_buffer_object." << endl;
        
    }
    
}


GLint initShaders()
{
    GLuint p, f, v;
    
	char *vs,*fs;
    
	v = glCreateShader(GL_VERTEX_SHADER);
	f = glCreateShader(GL_FRAGMENT_SHADER);
    
	// load shaders & get length of each
	GLint vlen;
	GLint flen;
	vs = loadFile("../src/vertexShader.glsl",vlen);
	fs = loadFile("../src/fragmentShader.glsl",flen);
	
	const char * vv = vs;
	const char * ff = fs;
    
	glShaderSource(v, 1, &vv,&vlen);
	glShaderSource(f, 1, &ff,&flen);
	
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
	
	p = glCreateProgram();
    
	glBindAttribLocation(p,0, "in_Position");
	glBindAttribLocation(p,1, "in_Color");
    
	glAttachShader(p,v);
	glAttachShader(p,f);
	
	glLinkProgram(p);
	glUseProgram(p);
    
    projMatrixLoc = glGetUniformLocation(p, "projMatrix");
    viewMatrixLoc = glGetUniformLocation(p, "viewMatrix");
    
	delete [] vs; // dont forget to free allocated memory
	delete [] fs; // we allocated this in the loadFile function...
    
    return p;
}


void idle(void)
{
    glutPostRedisplay();
}


void reshape(int w, int h)
{
    float ratio;
    
    if (h == 0)
        h = 1;
    
    // set viewport to be the entire window
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    
    ratio = (1.0f * w) / h;
    projMatrix = glm::perspective(53.13f, ratio, 1.0f, 30.0f);
    
    glUniformMatrix4fv(projMatrixLoc,  1, false, glm::value_ptr(projMatrix));
}


void display()
{
	//RGB(86,136,199)
	glClearColor(86.f/255.f,136.f/255.f,199.f/255.f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    viewMatrix = glm::lookAt(cameraEye,
                             glm::vec3(0,0,0),
                             cameraUp);
    
    glUniformMatrix4fv(viewMatrixLoc,  1, false, glm::value_ptr(viewMatrix));
    
    glBindVertexArray(vao);	// First VAO
	glDrawArrays(GL_TRIANGLES, 0, 36);	// draw first object
    
    glBindVertexArray(0);
    
    glutSwapBuffers();
    
}


void mouseCallback(int btn, int state, int x, int y)
{
	if(btn==GLUT_LEFT_BUTTON && state==GLUT_UP) //this is only for left up button events
        lastMouseX = NULL;
}

void mouseMotion(int x, int y)
{
    
    if (lastMouseX != NULL) {
        float xDesp = (lastMouseX - x) * CAMERA_DPP;
        float yDesp = (y - lastMouseY) * CAMERA_DPP;
        
        cameraEye = glm::rotateY(glm::rotateX(cameraEye, yDesp),xDesp);
        cameraUp = glm::rotateY(glm::rotateX(cameraUp, yDesp),xDesp);
    }
    
    lastMouseX = x;
    lastMouseY = y;
}

int main(int argc, char **argv)
{

    //Glu Init
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_3_2_CORE_PROFILE | GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(WINDOW_WIDTH,WINDOW_HEIGHT);
    glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - WINDOW_WIDTH)/2,
                           (glutGet(GLUT_SCREEN_HEIGHT) - WINDOW_HEIGHT)/2);
    glutCreateWindow("CUBE");
    
    glEnable(GL_DEPTH_TEST);
    
    //Glew Init
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
		cout << "glewInit failed, aborting." << endl;
		exit (1);
    }
    
    cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
    cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
    cout << "GLEW version: " << glewGetString(GLEW_VERSION) << endl;
    
    initVAO();
    program = initShaders();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutMouseFunc(mouseCallback);
    glutMotionFunc(mouseMotion);
        
    glutMainLoop();


    return 0;
}
