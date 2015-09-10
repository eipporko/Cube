#ifndef __CUBE__camera__
#define __CUBE__camera__

#include <stdio.h>

#include <glm/glm.hpp>

class Camera {
    
private:
    float initialFovy;
    glm::vec3 initialPosition;
    glm::vec3 initialUpVector;
    float initialNearClipping;
    float initialFarClipping;
    
    float fovy;
    glm::vec3 position;
    glm::vec3 lookVector;
    glm::vec3 upVector;
    float distanceToOrigin;
    float nearClipping;
    float farClipping;
    float rotationXAxis;
    float rotationYAxis;
    
public:
    
    static glm::mat4 projMatrix, viewMatrix;
    static glm::mat3 normalMatrix;
    static int h, w;
    static float n, f;
    static float top, bottom, right, left;
    
    Camera(glm::vec3 cameraPosition);
    
    //Getter & Setter
    void moveDistance(float amount);
    glm::vec3 getPosition() {return this->position; };
    
    void reset();
    void rotate(float amountX, float amountY);
    void update(int width, int height);
    
};

#endif