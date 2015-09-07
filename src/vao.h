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

class VAO
{

private:
    GLuint vaoID, vboID;
    bool initialized;
    int numOfVertices;
    int numOfTriangles;
    vector<glm::vec3> vertices; //DEPRECATED
    vector<glm::vec3> colors;   //DEPRECATED
    vector<glm::vec3> normals;  //DEPRECATED
    GLenum mode;
    typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudType;
    vector<float> radius;
    CloudType::Ptr cloud;
    
    vector<float> getRadius();
    glm::vec3 pickPoint(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3);
    
public:
    
    //Constructors
    VAO() { initialized = false; };
    VAO(int numOfVertices, int numOfTriangles, vector<glm::vec3>vertices, vector<glm::vec3>colors, vector<glm::vec3>normals, GLenum mode);
    VAO(CloudType::Ptr cloud);
    
    ~VAO() {}; //delete cloud };
    
    //Getters & Setters
    GLuint getVAOid() { return vaoID; };
    GLenum getMode() { return mode; };
    CloudType::Ptr getCloud() {return cloud; };
    
    bool isValid () { return initialized; };
    void pushToGPU();
    
    void sampleMesh(int samplesPerTriangle);
    void sampleSphere(int numOfSamples);
    
};

#endif
