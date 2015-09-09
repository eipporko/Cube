#ifndef __CUBE__camera__
#define __CUBE__camera__

#include <stdio.h>

#include <glm/glm.hpp>

class Camera {
    
private:
    float initialFovy = 53.13f;
    glm::vec3 initialPosition;
    glm::vec3 initialUpVector;
    float initialNearClipping = 0.1f;
    float initialFarClipping = 100;
    
    float fovy = 53.13f;
    glm::vec3 position;
    glm::vec3 lookVector;
    glm::vec3 upVector;
    float distanceToOrigin;
    float nearClipping = 0.1f;
    float farClipping = 100;
    float rotationXAxis = 0;
    float rotationYAxis = 0;
    
public:
    
    static glm::mat4 projMatrix, viewMatrix;
    static glm::mat3 normalMatrix;
    static int h, w;
    static float n, f;
    static float top, bottom, right, left;
    
    Camera(glm::vec3 cameraPosition) {
        this->distanceToOrigin = glm::length(cameraPosition);
        this->lookVector = glm::normalize(cameraPosition);
        this->upVector = glm::vec3(0,1,0);
        this->initialPosition = cameraPosition;
        this->initialUpVector = this->upVector;
    };
    
    //Getter & Setter
    void moveDistance(float amount);
    glm::vec3 getPosition() {return this->position; };
    
    void reset();
    void rotate(float amountX, float amountY);
    void update(int width, int height);
    
};

#endif