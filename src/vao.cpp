#include "vao.h"

/**
 @brief Load models into GPU memory.
 Members vaoID and vboID of the structure are overwriten with the references returned by GPU
 @param obj vao object
 @returns void
 */
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
                     sizeof(glm::vec3)*(obj->vertices.size() +
                                        obj->colors.size()+
                                        obj->normals.size()),
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
        
        // Put "normals" at an offset in the buffer equal to the filled size of
        // the buffer so far - i.e., sizeof(positions).
        glBufferSubData(GL_ARRAY_BUFFER,
                        sizeof(glm::vec3)*(obj->vertices.size()+
                                           obj->colors.size()),
                        sizeof(glm::vec3)*obj->normals.size(),
                        &obj->normals[0]);
        
        // Now "positions" is at offset 0 and "colors" is directly after it
        // in the same buffer.
        
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(sizeof(glm::vec3)*obj->vertices.size()));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(sizeof(glm::vec3)*(obj->vertices.size()+
                                                                                            obj->colors.size())));
        
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        
    }
    else
    {
        cout << "Video card does NOT support GL_ARB_vertex_buffer_object." << endl;
        
    }
    
}
