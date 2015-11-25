/*
 *
 * CUBE
 *
 * Copyright (c) David Antunez Gonzalez 2013-2015 <dantunezglez@gmail.com>
 * Copyright (c) Luis Omar Alvarez Mures 2013-2015 <omar.alvarez@udc.es>
 * Copyright (c) Emilio Padron Gonzalez 2013-2015 <emilioj@gmail.com>
 *
 * All rights reserved.
 *
 * This file is part of ToView.
 *
 * CUBE is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library.
 *
 */

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
    GLuint vaoID;
    vector<GLuint> vboID;
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
    
    
    int numOfVertexByVBO(int numOfVBO);
    int maxNumOfVertexByVBO();
    int numOfVBORequired(int numOfVertex);
    int getFreeVideoMemory();

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
    void draw();

    void sampleMesh(int samplesPerTriangle);
    void sampleSphere(int numOfSamples);

};

#endif
