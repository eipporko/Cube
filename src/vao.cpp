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

#include "vao.h"

#include <pcl/kdtree/kdtree_flann.h>

#define MAX_VBO_SIZE (1024*1024*3)


VAO::VAO(int numOfVertices, int numOfTriangles, vector<glm::vec3>vertices, vector<glm::vec3>colors, vector<glm::vec3>normals, GLenum mode)
{
    this->numOfVertices = numOfVertices;
    this->numOfTriangles = numOfTriangles;
    this->vertices = vertices;
    this->colors = colors;
    this->normals = normals;
    this->mode = mode;
    this->initialized = true;
}



VAO::VAO(CloudType::Ptr cloud)
{
    this->cloud = cloud;
    this->numOfVertices = cloud->points.size ();
    this->mode = GL_POINTS;
    this->initialized = true;
}


vector<float> VAO::getRadius()
{
    vector<float> radius;

    int K = 12;

    std::vector<glm::vec3> positions;
    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
    std::vector<int> k_indices;
    std::vector<float> k_distances;

    kdtree.setInputCloud(cloud->makeShared());

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    for (unsigned int i = 0; i < cloud->size(); i++) {

        if ( kdtree.nearestKSearch (cloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            radius.push_back(sqrt(pointNKNSquaredDistance[K-1]));


    }

    return radius;
}



/**
 @brief Gets a random point inside a triangle defined by its vertices
 http://parametricplayground.blogspot.com.es/2011/02/random-points-distributed-inside.html
 @param v1, v2, v3 vertices of an triangle
 @returns point
 */
glm::vec3 VAO::pickPoint(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3)
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


int VAO::getFreeVideoMemory()
{
    int availableKB[]={-1,-1,-1,-1};
    if(GLEW_NVX_gpu_memory_info)
    {
        glGetIntegerv(GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX,&availableKB[0]);
        printf("NVidia card\n");
    }

    if(GLEW_ATI_meminfo)
    {
        glGetIntegerv(GL_TEXTURE_FREE_MEMORY_ATI,availableKB);
        printf("ATI card\n");
    }
    return availableKB[0];
}



int VAO::numOfVertexByVBO(int numOfVBO)
{
    if (cloud->size() - (maxNumOfVertexByVBO() * numOfVBO) >= maxNumOfVertexByVBO())
        return maxNumOfVertexByVBO();
    else
        return cloud->size() - (maxNumOfVertexByVBO() * numOfVBO);
}


int VAO::maxNumOfVertexByVBO()
{
    return floor(MAX_VBO_SIZE/(sizeof(vaoVertex)*1.0f));
}



int VAO::numOfVBORequired(int numOfVertex)
{
    return ceil((numOfVertex*1.0f)/maxNumOfVertexByVBO());
}



void VAO::pushToGPU()
{
    vector<glm::vec3> vertices;
    vector<glm::vec3> colors;
    vector<glm::vec3> normals;

    vector<vaoVertex> vboData;

    if (GLEW_ARB_vertex_buffer_object)
    {
        cout << "Video card supports GL_ARB_vertex_buffer_object." << endl;

        // Allocate and assign a Vertex Array Object to our handle
        glGenVertexArrays(1, &vaoID);

        // Bind our Vertex Array Object as the current used object
        glBindVertexArray(vaoID);

        // Read cloud and get 3 vectors (vertices, colors and normals)
        if (cloud != NULL) {

            vector<float> radius = getRadius();
            
            int numberOfVBO = numOfVBORequired(cloud->size());
            
            cout << sizeof(vaoVertex)*cloud->size() << " bytes." << endl;
            cout << numberOfVBO << " VBO needed" << endl;

            for (unsigned int i = 0; i < cloud->size(); i++) {
                vaoVertex vertice;
                vertice.position = glm::vec3(cloud->points[i].x,
                                             cloud->points[i].y,
                                             cloud->points[i].z);

                vertice.normal = glm::vec3(cloud->points[i].normal_x,
                                           cloud->points[i].normal_y,
                                           cloud->points[i].normal_z);

                vertice.color = glm::vec3(cloud->points[i].r/255.f,
                                          cloud->points[i].g/255.f,
                                          cloud->points[i].b/255.f);
                vertice.radius = radius[i];

                vboData.push_back(vertice);
            }

            // Reserve a name for the buffer object.
            vboID.resize(numberOfVBO);
            glGenBuffers(numberOfVBO, &vboID[0]);
            
            for (int i=0; i < numberOfVBO; i++) {
                // Bind it to the GL_ARRAY_BUFFER target.
                glBindBuffer(GL_ARRAY_BUFFER, vboID[i]);
                
                int numberOfVertex = numOfVertexByVBO(i);
                
                glBufferData(GL_ARRAY_BUFFER,
                             sizeof(vaoVertex)*numberOfVertex,
                             &vboData[maxNumOfVertexByVBO()*i],
                             GL_STATIC_DRAW);
            }
            
            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);
            glEnableVertexAttribArray(2);
            glEnableVertexAttribArray(3);
            
        }

    }
    else
    {
        cout << "Video card does NOT support GL_ARB_vertex_buffer_object." << endl;

    }
}



void VAO::draw() {
    
    for (int i=0; i < vboID.size(); i++) {
        glBindBuffer(GL_ARRAY_BUFFER, vboID[i]);
        
        int nBufferSize = 0;
        glGetBufferParameteriv(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &nBufferSize);
        int originalVertexArraySize = ( nBufferSize / sizeof(vaoVertex) );
        
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vaoVertex), BUFFER_OFFSET(0));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vaoVertex), BUFFER_OFFSET(sizeof(glm::vec3)) );
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(vaoVertex), BUFFER_OFFSET(sizeof(glm::vec3)*2) );
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(vaoVertex), BUFFER_OFFSET(sizeof(glm::vec3)*3) );
        
        glDrawArrays(mode, 0, originalVertexArraySize);
    }
}



void VAO::sampleMesh(int samplesPerTriangle) {

    if (mode == GL_TRIANGLES) {

        this->numOfVertices = 0;

        CloudType::Ptr cloud (new CloudType);

        cloud->width = this->numOfTriangles * samplesPerTriangle;
        cloud->height  = 1;
        cloud->points.resize(cloud->width * cloud->height);

        int line;
        for (int i = 0; i < this->numOfTriangles; i++) {
            for (int j = 0; j < samplesPerTriangle; j++) {
                line = i*3;

                glm::vec3 point = pickPoint(this->vertices[line], this->vertices[line+1], this->vertices[line+2]);
                cloud->points[this->numOfVertices].x = point.x;
                cloud->points[this->numOfVertices].y = point.y;
                cloud->points[this->numOfVertices].z = point.z;
                cloud->points[this->numOfVertices].normal_x = this->normals[i].x;
                cloud->points[this->numOfVertices].normal_y = this->normals[i].y;
                cloud->points[this->numOfVertices].normal_z = this->normals[i].z;
                cloud->points[this->numOfVertices].r = this->colors[line].r * 255;
                cloud->points[this->numOfVertices].g = this->colors[line].g * 255;
                cloud->points[this->numOfVertices].b = this->colors[line].b * 255;

                this->numOfVertices++;
            }
        }

        this->cloud = cloud;

        this->mode = GL_POINTS;

    }

}

void VAO::sampleSphere(int numOfSamples)
{
    double x, y, z;

    this->numOfTriangles = 0;
    this->numOfVertices = 0;

    CloudType::Ptr cloud (new CloudType);

    cloud->width = numOfSamples;
    cloud->height  = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (int i = 0; i < numOfSamples; i += 2) {
        //x^2 + y^2 + z^2 = 1

        x = rand() / double(RAND_MAX) * 2 - 1;
        y = rand() / double(RAND_MAX) * 2 - 1;

        while (sqrt(pow(x, 2) + pow(y, 2)) > 1.0)
            y = rand() / double(RAND_MAX) * 2 - 1;

            z = sqrt(1 - pow(x, 2) - pow(y, 2));

            cloud->points[i].x = x;
            cloud->points[i].y = y;
            cloud->points[i].z = z;
            cloud->points[i].normal_x = x;
            cloud->points[i].normal_y = y;
            cloud->points[i].normal_z = z;
            cloud->points[i].r = 0;
            cloud->points[i].g = 0;
            cloud->points[i].b = 255;

            cloud->points[i+1].x = x;
            cloud->points[i+1].y = y;
            cloud->points[i+1].z = -z;
            cloud->points[i+1].normal_x = x;
            cloud->points[i+1].normal_y = y;
            cloud->points[i+1].normal_z = -z;
            cloud->points[i+1].r = 255;
            cloud->points[i+1].g = 0;
            cloud->points[i+1].b = 0;

            this->numOfVertices = this->numOfVertices + 2;

            }


    this->cloud = cloud;

    this->mode = GL_POINTS;
}
