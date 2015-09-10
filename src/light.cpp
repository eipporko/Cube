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

#include "light.h"

vector<Light*> Light::lights;
float Light::lightPosition[MAX_LIGHTS*3];
float Light::lightColor[MAX_LIGHTS*3];
float Light::lightIntensity[MAX_LIGHTS];
int Light::lightCount;


Light::Light(glm::vec3 position, glm::vec3 color, float intensity) {
    this->initialPosition = position;
    this->position = position;
    this->color = color;
    this->intensity = intensity;
    
    lights.push_back(this);
 }

Light::~Light()
{
    auto iter = find(lights.begin(), lights.end(), this);
    
    if (iter != lights.end())
        lights.erase(iter);
}


void Light::pushToGPU()  {
    vector<Light *> list;
    list.push_back(this);
    Light::pushToGPU(list);
};


void Light::resetAll() {
    for (int i = 0; i < lights.size(); i++)
        lights[i]->position = lights[i]->initialPosition;
}


void Light::pushToGPU(vector<Light*> listOfLights) {
    
    int counter = 0;
    for (int i = 0; i < listOfLights.size()*3 ; i = i+3){
        lightPosition[i] = listOfLights[counter]->position.x;
        lightPosition[i+1] = listOfLights[counter]->position.y;
        lightPosition[i+2] = listOfLights[counter]->position.z;
        
        lightColor[i] = listOfLights[counter]->color.r;
        lightColor[i+1] = listOfLights[counter]->color.g;
        lightColor[i+2] = listOfLights[counter]->color.b;
        
        lightIntensity[counter] = listOfLights[counter]->intensity;
        
        counter++;
    }
    
    lightCount = listOfLights.size();
    
}