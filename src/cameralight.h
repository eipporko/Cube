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

#ifndef __CUBE__cameralight__
#define __CUBE__cameralight__

#include <iostream>

#include <GL/glew.h>

#include "light.h"
#include "camera.h"

using namespace std;

class CameraLight : public Light {

private:
    Camera* cameraAttached;
    
public:
    
    //Constructor
    CameraLight(Camera* camera, glm::vec3 color, float intensity) : Light(glm::vec3(0,0,0), color, intensity) {
        this->cameraAttached = camera;
    };
    
    void update();
    
};

#endif /* defined(__CUBE__cameralight__) */
