#include <iostream>
#include <GL/glew.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#elif
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif


#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

void draw(void)
{
	// Black background
	glClearColor(86.f/255.f,136.f/255.f,199.f/255.f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //Draw
    glFlush();
 
}

void init(int argc, char **argv)
{
    //Glu Init
    glutInit(&argc, argv);
    //RGBA Color and Single Buffer
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE | GLUT_DEPTH);
    
    //Center Window
    glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - WINDOW_WIDTH)/2,
                           (glutGet(GLUT_SCREEN_HEIGHT) - WINDOW_HEIGHT)/2);
    
    //Window Size
    glutInitWindowSize(WINDOW_WIDTH,WINDOW_HEIGHT);
    
    //Create Window
    glutCreateWindow("CUBE");
    
    //Glew Init
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        /* Problem: glewInit failed, something is seriously wrong. */
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    }
    
    //Call to the drawing function
    glutDisplayFunc(draw);
    
    // Loop require by OpenGL
    glutMainLoop();

}

int main(int argc, char **argv)
{
    init(argc,argv);
    return 0;
}
