#include "vao.h"

#include <pcl/kdtree/kdtree_flann.h>


// K nearest neighbor search
vector<float> getRadius(struct vao *obj)
{
    vector<float> radius;
    
    int K = 12;

    std::vector<glm::vec3> positions;
    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
    std::vector<int> k_indices;
    std::vector<float> k_distances;
    
    kdtree.setInputCloud(obj->cloud->makeShared());
    
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    
    for (unsigned int i = 0; i < obj->cloud->size(); i++) {
        
        if ( kdtree.nearestKSearch (obj->cloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            radius.push_back(sqrt(pointNKNSquaredDistance[K-1]));

        
    }

    
    return radius;
}

/**
 @brief Load models into GPU memory.
 Members vaoID and vboID of the structure are overwriten with the references returned by GPU
 @param obj vao object
 @returns void
 */
void loadVAO(struct vao *obj)
{
    vector<glm::vec3> vertices;
    vector<glm::vec3> colors;
    vector<glm::vec3> normals;
    
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
        
        // Read cloud and get 3 vectors (vertices, colors and normals)
        if (obj->cloud != NULL)
            for (unsigned int i = 0; i < obj->cloud->size(); i++) {
                vertices.push_back( glm::vec3( obj->cloud->points[i].x,
                                               obj->cloud->points[i].y,
                                               obj->cloud->points[i].z) );
                
                normals.push_back( glm::vec3( obj->cloud->points[i].normal_x,
                                              obj->cloud->points[i].normal_y,
                                              obj->cloud->points[i].normal_z) );
                
                colors.push_back(glm::vec3( obj->cloud->points[i].r/255.f,
                                            obj->cloud->points[i].g/255.f,
                                            obj->cloud->points[i].b/255.f));
            }
        
        vector<float> radius = getRadius(obj);
        
        // Allocate space for it (sizeof(vertices) + sizeof(colors)).
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(glm::vec3)*(vertices.size() +
                                        colors.size()+
                                        normals.size()) +
                     sizeof(float) * radius.size(),
                     NULL,
                     GL_STATIC_DRAW);
        
        // Put "vertices" at offset zero in the buffer.
        glBufferSubData(GL_ARRAY_BUFFER,
                        0,
                        sizeof(glm::vec3)*vertices.size(),
                        &vertices[0]);
        
        // Put "colors" at an offset in the buffer equal to the filled size of
        // the buffer so far - i.e., sizeof(positions).
        glBufferSubData(GL_ARRAY_BUFFER,
                        sizeof(glm::vec3)*vertices.size(),
                        sizeof(glm::vec3)*colors.size(),
                        &colors[0]);
        
        // Put "normals" at an offset in the buffer equal to the filled size of
        // the buffer so far - i.e., sizeof(positions).
        glBufferSubData(GL_ARRAY_BUFFER,
                        sizeof(glm::vec3)*(vertices.size()+
                                           colors.size()),
                        sizeof(glm::vec3)*normals.size(),
                        &normals[0]);
        
        // Put "radius" at an offset in the buffer equal to the filled size of ...
        glBufferSubData(GL_ARRAY_BUFFER,
                        sizeof(glm::vec3)*(vertices.size() +
                                           colors.size()+
                                           normals.size()),
                        sizeof(float)*radius.size(),
                        &radius[0]);
        
        //cout << "###########################################" << endl;
        //for (unsigned int i = 0; i < radius.size(); i++) {
        //    cout << radius[i] << endl;
        //}
        
        // Now "positions" is at offset 0 and "colors" is directly after it
        // in the same buffer.
        
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(sizeof(glm::vec3)*vertices.size()));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(sizeof(glm::vec3)*(vertices.size()+
                                                                                            colors.size())));
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(sizeof(glm::vec3)*(vertices.size() +
                                                                                             colors.size()+
                                                                                             normals.size()) ));
        
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glEnableVertexAttribArray(3);
        
    }
    else
    {
        cout << "Video card does NOT support GL_ARB_vertex_buffer_object." << endl;
        
    }
    
}
