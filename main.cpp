#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <GL/glew.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#elif
#include <GL/glut.h>
#endif

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480
#define M_PI 3.14159265358979323846


using namespace std;


//Globals
GLuint vao, vbo[2];

GLuint projMatrixLoc, viewMatrixLoc;

// storage for Matrices
float projMatrix[16];
float viewMatrix[16];

GLint program;

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
    
// color array
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


void crossProduct(float *a, float *b, float *res)
{
    res[0] = a[1] * b[2] - b[1] * a[2];
    res[1] = a[2] * b[0] - b[2] * a[0];
    res[2] = a[0] * b[1] - b[0] * a[1];
}

void normalize(float *a)
{
    float mag = sqrt(a[0] * a [0] + a[1] * a[1] + a[2] * a[2]);
    
    a[0] /=  mag;
    a[1] /=  mag;
    a[2] /=  mag;
}

void setIdentityMatrix(float *mat, int size)
{
    for (int i = 0; i < size * size; i++)
        mat[i] = 0.0f;
        
    for (int i = 0; i < size; i++)
        mat[i + i * size] = 1.0f;
}

void multMatrix(float *a, float *b)
{
    float res[16];
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            res[j*4 + i] = 0.0f;
            for (int k = 0; k < 4; k++) {
                res[j*4 + i] += a[k*4 + i] * b[j*4 + k];
            }
        }
    }
    
    memcpy(a, res, 16 * sizeof(float));
}

void setTranslationMatrix(float *mat, float x, float y, float z)
{
    setIdentityMatrix(mat, 4);
    
    mat[12] = x;
    mat[13] = y;
    mat[14] = z;
}

void buildProjectionMatrix(float fov, float ratio, float nearP, float farP)
{
    float f = 1.0f / tan (fov * (M_PI / 360.0));
    
    setIdentityMatrix(projMatrix, 4);
    
    projMatrix[0] = f / ratio;
    projMatrix[1 * 4 + 1] = f;
    projMatrix[2 * 4 + 2] = (farP + nearP) / (nearP - farP);
    projMatrix[3 * 4 + 2] = (2.0f * farP * nearP) / (nearP - farP);
    projMatrix[2 * 4 + 3] = -1.0f;
    projMatrix[3 * 4 + 3] = 0.0f;
}

void setCamera(float posX, float posY, float posZ,
               float lookAtX, float lookAtY, float lookAtZ)
{
    float right[3];
    
    float up[] = {0.0f, 1.0f, 0.0f};
    float dir[] = {lookAtX - posX, lookAtY - posY, lookAtZ - posZ};
    normalize(dir);
    
    crossProduct(dir, up, right);
    normalize(right);
    
    crossProduct(right, dir, up);
    normalize(up);
    
    float aux[16];
    
    viewMatrix[0] = right[0];
    viewMatrix[4] = right[1];
    viewMatrix[8] = right[2];
    viewMatrix[12] = 0.0f;
    
    viewMatrix[1] = up[0];
    viewMatrix[5] = up[1];
    viewMatrix[9] = up[2];
    viewMatrix[13] = 0.0f;
    
    viewMatrix[2] = -dir[0];
    viewMatrix[6] = -dir[1];
    viewMatrix[10] = -dir[2];
    viewMatrix[14] = 0.0f;
    
    viewMatrix[3] = 0.0f;
    viewMatrix[7] = 0.0f;
    viewMatrix[11] = 0.0f;
    viewMatrix[15] = 1.0f;
    
    setTranslationMatrix(aux, -posX, -posY, -posZ);
    
    multMatrix(viewMatrix, aux);
    
    glUniformMatrix4fv(projMatrixLoc,  1, false, projMatrix);
    glUniformMatrix4fv(viewMatrixLoc,  1, false, viewMatrix);
    
}

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
    //DEBUG
    char *path=NULL;
    size_t size;
    path=getcwd(path,size);
    cout << "current Path" << path << endl;
    //DEBUG
    
    GLuint p, f, v;
    
	char *vs,*fs;
    
	v = glCreateShader(GL_VERTEX_SHADER);
	f = glCreateShader(GL_FRAGMENT_SHADER);
    
	// load shaders & get length of each
	GLint vlen;
	GLint flen;
	vs = loadFile("../../vertexShader.glsl",vlen);
	fs = loadFile("../../fragmentShader.glsl",flen);
	
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
    buildProjectionMatrix(53.13f, ratio, 1.0, 30.0f);
    
}


void setCamera()
{
    //Do Things

    //glUniformMatrix4fv(projMatrixLoc,  1, false, projMatrix);
    glUniformMatrix4fv(viewMatrixLoc,  1, false, viewMatrix);
}


void display()
{
	//RGB(86,136,199)
	glClearColor(86.f/255.f,136.f/255.f,199.f/255.f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    setCamera(3,2,-7,0,0,0);
    
    glBindVertexArray(vao);	// First VAO
	glDrawArrays(GL_TRIANGLES, 0, 36);	// draw first object
    
    glBindVertexArray(0);
    
    glutSwapBuffers();
    
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
        
    glutMainLoop();


    return 0;
}
