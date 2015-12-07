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

#include "debugcameracallback.h"

#include <glm/trigonometric.hpp>

#include <iostream>

#define A 1.6f
#define B 5.0f

#define STEPS_PER_REV 200
#define DEGREES_PER_STEP 360.0f/STEPS_PER_REV
#define REVS_BY_SHADER 4

using namespace std;

bool DebugCameraCallback::move() {
    
    //http://math.stackexchange.com/questions/315386/ellipse-in-polar-coordinates
    float distance = sqrt(pow(B * cos( glm::radians(angle) ), 2) + pow(A * sin( glm:: radians(angle) ), 2));
    
    cameraAttached->setDistance(distance);
    cameraAttached->rotate(DEGREES_PER_STEP, 0);
    
    angle += DEGREES_PER_STEP;
    steps++;

    if ( fmod(steps, STEPS_PER_REV * REVS_BY_SHADER)  == 0.0f)
        return true;
    else
        return false;
}


void DebugCameraCallback::operation() {
    
    if (move())
        cout << "AHORA CAMBIO DE SHADER" << endl;
    
    
}