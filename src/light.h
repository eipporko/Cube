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

#ifndef __CUBE__light__
#define __CUBE__light__

#include <iostream>
#include <vector>
#include <algorithm>

#include <glm/glm.hpp>

#define MAX_LIGHTS 16

using namespace std;

struct light {
    glm::vec4 position;
    glm::vec4 color;
    float intensity;
};

class Light{

private:
    static vector<Light*> lights;

protected:
    glm::vec3 initialPosition;
    glm::vec3 position;
    glm::vec3 color;
    float intensity;

public:
    static float lightPosition[MAX_LIGHTS*3];
    static float lightColor[MAX_LIGHTS*3];
    static float lightIntensity[MAX_LIGHTS];
    static int lightCount;

    Light(glm::vec3 position, glm::vec3 color, float intensity);

    ~Light();

    //Getters & Setters
    void setPosition(glm::vec3 newPosition) { this->position = newPosition; };
    void setIntensity(float intensity) { this->intensity = intensity; };
    void setColor(glm::vec3 color) { this->color = color; };

    virtual void update() {};
    void pushToGPU();

    static int maxLights() {return MAX_LIGHTS; };

    static void resetAll();

    static void pushAllToGPU() { pushToGPU(lights); };

    static void pushToGPU(vector<Light*> listOfLights);

};

#endif /* defined(__CUBE__light__) */
