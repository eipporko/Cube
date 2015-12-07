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

#include "camera.h"

#include <GL/glew.h>

#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

Camera* Camera::activeCamera;
glm::mat4 Camera::projMatrix, Camera::viewMatrix;
glm::mat3 Camera::normalMatrix;
int Camera::h, Camera::w;
float Camera::n, Camera::f;
float Camera::top, Camera::bottom, Camera::right, Camera::left;

Camera::Camera(glm::vec3 cameraPosition) {

	this->initialFovy = 53.13f;
	this->initialNearClipping = 0.1f;
	this->initialFarClipping = 100;
	this->fovy = 53.13f;
	this->nearClipping = 0.1f;
    this->farClipping = 100;
    this->rotationXAxis = 0;
    this->rotationYAxis = 0;
	this->distanceToOrigin = glm::length(cameraPosition);
	this->lookVector = glm::normalize(cameraPosition);
	this->upVector = glm::vec3(0,1,0);
	this->initialPosition = cameraPosition;
	this->initialUpVector = this->upVector;

};


void Camera::moveDistance(float amount) {
    this->distanceToOrigin += amount;
};


void Camera::setDistance(float distance) {
    this->distanceToOrigin = distance;
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


void Camera::setUpdateCallback(CameraCallback* callback) {
    this->callback = callback;
    if (callback != NULL)
        callback->setCamera(this);
}


void Camera::rotate(float amountX, float amountY) {
    
    glm::vec3 rightVector = glm::cross(this->lookVector, this->upVector);
    
    this->lookVector = glm::rotate(this->lookVector, glm::radians(amountX), this->upVector );
    this->lookVector = glm::rotate(this->lookVector, glm::radians(amountY), rightVector );
    
    this->upVector = glm::rotate(this->upVector, glm::radians(amountY), rightVector );
};


void Camera::updateView(int width, int height) {
    
    this->position = this->lookVector * distanceToOrigin;
    
    Camera::viewMatrix = glm::lookAt(this->position,
                             glm::vec3(0,0,0),
                             this->upVector);
    
    Camera::normalMatrix = glm::inverseTranspose(glm::mat3(Camera::viewMatrix));
    
    if (height == 0)
        height = 1;
    
    
    float ratio;
    ratio = (1.0f * width) / height;
    
    Camera::projMatrix = glm::perspective(glm::radians(this->fovy), ratio, this->nearClipping, this->farClipping);
    
    GLfloat realTop = (GLfloat) tan( 0.5f * glm::radians(this->fovy)) * this->nearClipping;
    
    Camera::n = this->nearClipping;
    Camera::f = this->farClipping;
    Camera::h = height;
    Camera::w = width;
    Camera::top = -realTop;
    Camera::bottom = realTop;
    Camera::left = -realTop * ratio;
    Camera::right = realTop * ratio;
};


void Camera::update(int width, int height) {
    if (callback != NULL) {
        callback->operation();
        updateView(width, height);
    }
}


