#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif


#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

#define POINT_SIZE 2
#define POINTS_PER_TRIANGLE 1500

#define CAMERA_RPP 2*M_PI/1000.0 //resolution 1000px = 2PI

#define BUFFER_OFFSET(bytes) ((GLubyte*) NULL + (bytes))
#define SGN(x)   (((x) < 0) ? (-1) : (1))
#define DEG_TO_RAD(x) (x * (M_PI / 180.0))
#define LESS_THAN(x, limit) ((x > limit) ? (limit) : (x))
#define GREATER_THAN(x, limit) ((x < limit) ? (limit) : (x))


using namespace std;


//Projection and View Matrix
GLint projMatrixLoc, viewMatrixLoc; //uniform locations
glm::mat4 projMatrix, viewMatrix;   //transformation matrix


//Camera
float cameraDistance = 7.0f;
glm::vec3 cameraEye = glm::vec3(0, 0, -cameraDistance);
glm::vec3 cameraUp = glm::vec3(0,1,0);
float cameraAngleX, cameraAngleY;

//Mouse
int lastMouseX = NULL, lastMouseY = NULL; //last mouse position pressed;

//VAO Struct Definition
struct vao {
    GLuint vaoID, vboID;
    int numOfVertices = 0;
    int numOfTriangles = 0;
    vector<glm::vec3> vertices;
    vector<glm::vec3> colors;
    GLenum mode;
};

//Pointer to the VAO to be rendered in display func
vao* displayVAO = NULL;

//Cube Description
//TODO: struct vao initilization with this data
//
//    v1----v3
//   /|     /|
//  v2+---v4 |
//  | |    | |
//  | v5---+v7
//  |/     |/
//  v6----v8

glm::vec3 v1 = glm::vec3(-1,1,1);
glm::vec3 v2 = glm::vec3(-1,1,-1);
glm::vec3 v3 = glm::vec3(1,1,1);
glm::vec3 v4 = glm::vec3(1,1,-1);
glm::vec3 v5 = glm::vec3(-1,-1,1);
glm::vec3 v6 = glm::vec3(-1,-1,-1);
glm::vec3 v7 = glm::vec3(1,-1,1);
glm::vec3 v8 = glm::vec3(1,-1,-1);

glm::vec3 blue  =   glm::vec3(0,0,1);
glm::vec3 green =   glm::vec3(0,1,0);
glm::vec3 cyan  =   glm::vec3(0,1,1);
glm::vec3 red   =   glm::vec3(1,0,0);
glm::vec3 magenta = glm::vec3(1,0,1);
glm::vec3 yellow =  glm::vec3(1,1,0);

glm::vec3 vertices[] = {v1, v2, v3, //Top
                        v3, v2, v4,
    
                        v4, v2, v6, //Front
                        v6, v8, v4,

                        v4, v8, v3, //Right
                        v3, v8, v7,

                        v7, v8, v5, //Bottom
                        v5, v8, v6,

                        v6, v2, v5, //Left
                        v5, v2, v1,

                        v1, v3, v7, //Back
                        v7, v5, v1};

glm::vec3 colors[] = {  blue, blue, blue,   //Top
                        blue, blue, blue,
    
                        green, green, green,  //Front
                        green, green, green,
    
                        cyan, cyan, cyan,   //Right
                        cyan, cyan, cyan,
    
                        red, red, red,    //Bottom
                        red, red, red,
    
                        magenta, magenta, magenta,//Left
                        magenta, magenta, magenta,
    
                        yellow, yellow, yellow, //Back
                        yellow, yellow, yellow };




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


// http://parametricplayground.blogspot.com.es/2011/02/random-points-distributed-inside.html
glm::vec3 pickPoint(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3)
{
    glm::vec3 point;
    
    double c, a, b;
    
    a = rand() / double(RAND_MAX);
    b = rand() / double(RAND_MAX);
    
    if (a + b > 1)
    {
        a = 1.0f - a;
        b = 1.0f - b;
    }
    c = 1.0f - a - b;
    
    point.x = (a * v1.x) + (b * v2.x) + (c * v3.x);
    point.y = (a * v1.y) + (b * v2.y) + (c * v3.y);
    point.z = (a * v1.z) + (b * v2.z) + (c * v3.z);
    
    return point;
}


//TODO: *mesh param... hello?
vao sampleMesh(vao *mesh)
{
    vao sampledMesh;
    
    int line;
    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < POINTS_PER_TRIANGLE; j++) {
            line = i*3;
            sampledMesh.vertices.push_back(pickPoint(vertices[line], vertices[line+1], vertices[line+2]));
            sampledMesh.colors.push_back(colors[line]);
            sampledMesh.numOfVertices++;
        }
    }
    
    sampledMesh.mode = GL_POINTS;
    
    return sampledMesh;
}


void loadVAO(struct vao *obj)
{
    
    if (GLEW_ARB_vertex_buffer_object)
    {
        cout << "Video card supports GL_ARB_vertex_buffer_object." << endl;
        
        // Allocate and assign a Vertex Array Object to our handle
        glGenVertexArrays(1, &obj->vaoID);
        
        // Bind our Vertex Array Object as the current used object
        glBindVertexArray(obj->vaoID);
        
        // Reserve a name for the buffer object.
        glGenBuffers(1, &obj->vboID);
        
        // Bind it to the GL_ARRAY_BUFFER target.
        glBindBuffer(GL_ARRAY_BUFFER, obj->vboID);
        
        // Allocate space for it (sizeof(vertices) + sizeof(colors)).
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(glm::vec3)*(obj->vertices.size() + obj->colors.size()),
                     NULL,
                     GL_STATIC_DRAW);
        
        // Put "vertices" at offset zero in the buffer.
        glBufferSubData(GL_ARRAY_BUFFER,
                        0,
                        sizeof(glm::vec3)*obj->vertices.size(),
                        &obj->vertices[0]);
        
        // Put "colors" at an offset in the buffer equal to the filled size of
        // the buffer so far - i.e., sizeof(positions).
        glBufferSubData(GL_ARRAY_BUFFER,
                        sizeof(glm::vec3)*obj->vertices.size(),
                        sizeof(glm::vec3)*obj->colors.size(),
                        &obj->colors[0]);
        
        // Now "positions" is at offset 0 and "colors" is directly after it
        // in the same buffer.
        
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(sizeof(glm::vec3)*obj->vertices.size()));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        
        displayVAO = obj;
    }
    else
    {
        cout << "Video card does NOT support GL_ARB_vertex_buffer_object." << endl;
        
    }
    
}


void initShaders()
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
    
    if (displayVAO != NULL) {
        glBindVertexArray(displayVAO->vaoID);
        glDrawArrays(displayVAO->mode, 0, displayVAO->numOfVertices);
    }
    
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
        cameraAngleX += (lastMouseX - x) * CAMERA_RPP;
        cameraAngleY += (lastMouseY - y) * CAMERA_RPP;
        
        //-PI/2 < cameraAngleY < PI/2
        cameraAngleY = LESS_THAN(cameraAngleY, M_PI/2.0f);
        cameraAngleY = GREATER_THAN(cameraAngleY, -M_PI/2.0f);

        // Calculate the camera position using the distance and angles
        cameraEye.x = cameraDistance * -sinf(cameraAngleX) * cosf(cameraAngleY);
        cameraEye.y = cameraDistance * -sinf(cameraAngleY);
        cameraEye.z = -cameraDistance * cosf(cameraAngleX) * cosf(cameraAngleY);
    }
    
    lastMouseX = x;
    lastMouseY = y;
}


void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27: //esc
            exit (0);
            break;
    }
}


int main(int argc, char **argv)
{
    
    //Glu Init
    glutInit(&argc, argv);
    
    #ifdef __APPLE__
        glutInitDisplayMode(GLUT_3_2_CORE_PROFILE | GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    #else
        // Freeglut
        glutInitContextVersion(3, 2);
        glutInitContextProfile(GLUT_CORE_PROFILE);
        glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    #endif
    
    glutInitWindowSize(WINDOW_WIDTH,WINDOW_HEIGHT);
    glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - WINDOW_WIDTH)/2,
                           (glutGet(GLUT_SCREEN_HEIGHT) - WINDOW_HEIGHT)/2);
    glutCreateWindow("CUBE");
    
    glEnable(GL_DEPTH_TEST);
    glPointSize(POINT_SIZE);
    
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
    
    vao pointCloudCube;
    pointCloudCube = sampleMesh(NULL);
    loadVAO(&pointCloudCube);
    
    initShaders();
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouseCallback);
    glutMotionFunc(mouseMotion);
    
    glutMainLoop();

    
    return 0;
}
