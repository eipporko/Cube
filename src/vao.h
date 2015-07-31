#ifndef __CUBE__vao__
#define __CUBE__vao__

#include <iostream>
#include <vector>

#include <GL/glew.h>
#include <glm/glm.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define BUFFER_OFFSET(bytes) ((GLubyte*) NULL + (bytes))

using namespace std;


struct vaoVertex {
    glm::vec3 position;
    glm::vec3 color;
    glm::vec3 normal;
    float radius;
};

//VAO Struct Definition
/** @struct vao
 *  @brief This structure blah blah blah...
 *  @var vao::vaoID
 *  Member 'vaoID' contains vao reference
 *  @var vao::vboID
 *  Member 'vboID' contains vbo reference
 *  @var vao::numOfVertices
 *  Member 'numOfVertices' contains the number of vertices
 *  @var vao::numOfTriangles
 *  Member 'numOfTriangles' contains the number of triangles
 *  @var vao::vertices
 *  Member 'vertices' contains a list with vertices
 *  @var vao::colors
 *  Member 'colors' contains a list with a colour per vertex
 *  @var vao::normals
 *  Member 'normals' contains a list with normal vector per vertex
 *  @var vao::mode
 *  Member 'mode' contains GLmode (GL_TRIANGLES | GL_POINTS)
 */
struct vao {
    GLuint vaoID, vboID;
    int numOfVertices;
    int numOfTriangles;
    vector<glm::vec3> vertices; //DEPRECATED
    vector<glm::vec3> colors;   //DEPRECATED
    vector<glm::vec3> normals;  //DEPRECATED
    GLenum mode;
    typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudType;
    vector<float> radius;
    CloudType::Ptr cloud;
};



vector<float> getRadius(struct vao *obj);

/**
 @brief Load models into GPU memory.
 Members vaoID and vboID of the structure are overwriten with the references returned by GPU
 @param obj vao object
 @returns void
 */
void loadVAO(struct vao *obj);

#endif
