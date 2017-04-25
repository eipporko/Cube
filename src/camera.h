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

#ifndef __CUBE__camera__
#define __CUBE__camera__

#include <stdio.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>

class CameraCallback;

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

    CameraCallback* callback = NULL;

public:
    static Camera* activeCamera;
    static glm::mat4 projMatrix, viewMatrix;
    static glm::mat3 normalMatrix;
    static int h, w;
    static float n, f;
    static float top, bottom, right, left;

    Camera(glm::vec3 cameraPosition);

    //Getter & Setter
    void moveDistance(float amount);
    void setDistance(float distance);
    glm::vec3 getPosition() {return this->position; };

    void setActive() { activeCamera = this; };
    void setUpdateCallback(CameraCallback* callback);
    void reset();
    void rotate(float amountX, float amountY);
    void updateView(int width, int height);
    void update(int width, int height);

};


class CameraCallback {

protected:
    Camera* cameraAttached = NULL;

public:
    void setCamera(Camera* cam) { cameraAttached = cam; };
    Camera* getCamera() {return cameraAttached; };
    virtual void operation() {};

};

#endif
