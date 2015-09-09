#include "camera.h"

#include <GL/glew.h>

#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#define DEG_TO_RAD(x) (x * (M_PI / 180.0))

glm::mat4 Camera::projMatrix, Camera::viewMatrix;
glm::mat3 Camera::normalMatrix;
int Camera::h, Camera::w;
float Camera::n, Camera::f;
float Camera::top, Camera::bottom, Camera::right, Camera::left;

void Camera::moveDistance(float amount) {
    this->distanceToOrigin += amount;
};

void Camera::reset() {
    this->fovy = this->initialFovy;
    this->distanceToOrigin = glm::length(this->initialPosition);
    this->lookVector = glm::normalize(this->initialPosition);
    this->upVector = this->initialUpVector;
    this->nearClipping = this->initialNearClipping;
    this->farClipping = this->initialFarClipping;
    this->rotationXAxis = 0;
    this->rotationYAxis = 0;
};


void Camera::rotate(float amountX, float amountY) {
    this->lookVector = glm::rotate(this->lookVector, amountX, glm::vec3(0,1,0) );
    this->lookVector = glm::rotate(this->lookVector, amountY, glm::vec3(1,0,0) );
    
    this->upVector = glm::rotate(this->upVector, amountX, glm::vec3(0,1,0) );
    this->upVector = glm::rotate(this->upVector, amountY, glm::vec3(1,0,0) );
};


void Camera::update(int width, int height) {
    
    this->position = this->lookVector * distanceToOrigin;
    
    Camera::viewMatrix = glm::lookAt(this->position,
                             glm::vec3(0,0,0),
                             this->upVector);
    
    Camera::normalMatrix = glm::inverseTranspose(glm::mat3(Camera::viewMatrix));
    
    if (height == 0)
        height = 1;
    
    
    float ratio;
    ratio = (1.0f * width) / height;
    
    Camera::projMatrix = glm::perspective(this->fovy, ratio, this->nearClipping, this->farClipping);
    
    GLfloat realTop = (GLfloat) tan( 0.5f * DEG_TO_RAD(this->fovy)) * this->nearClipping;
    
    Camera::n = this->nearClipping;
    Camera::f = this->farClipping;
    Camera::h = height;
    Camera::w = width;
    Camera::top = -realTop;
    Camera::bottom = realTop;
    Camera::left = -realTop * ratio;
    Camera::right = realTop * ratio;
};



