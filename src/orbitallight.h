#ifndef __CUBE__light__
#define __CUBE__light__

#include <iostream>
#include <vector>

#include <GL/glew.h>

#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "shader.h"

#define MAX_LIGHTS 10

using namespace std;

struct light {
    glm::vec4 position;
    glm::vec4 color;
    float intensity;
};

class OrbitalLight {

private:
    static vector<OrbitalLight*> lights;
    
    glm::vec3 initialPosition;
    glm::vec3 position;
    glm::vec3 color;
    glm::vec3 axisRotation;
    float stepRotation;
    float intensity;
public:

    static float lightPosition[MAX_LIGHTS*3];
    static float lightColor[MAX_LIGHTS*3];
    static float lightIntensity[MAX_LIGHTS];
    static int lightCount;
    
    //Constructor
    OrbitalLight(int intensity) {
        this->intensity = intensity;
        this->color = glm::vec3(1,1,1);
        lights.push_back(this);
    };
    OrbitalLight(glm::vec3 position, glm::vec3 color, float intensity, glm::vec3 axis, float step) {
        this->initialPosition = position;
        this->position = position;
        this->color = color;
        this->intensity = intensity;
        this->axisRotation = axis;
        this->stepRotation = step;
        
        lights.push_back(this);
    };
    
    ~OrbitalLight()
    {
        auto iter = find(lights.begin(), lights.end(), this);
        
        if (iter != lights.end())
            lights.erase(iter);
    }
    
    
    //Getters & Setters
    void setPosition(glm::vec3 newPosition) { this->position = newPosition; };
    
    
    void update();
    void pushToGPU();
    
    static int maxLights() {return MAX_LIGHTS; };
    
    static void resetAll() {
        for (int i = 0; i < lights.size(); i++)
            lights[i]->position = lights[i]->initialPosition;
    }
    static void pushAllToGPU() { pushToGPU(lights); };
    
    static void pushToGPU(vector<OrbitalLight*> listOfLights) {

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
        
    };
    
};

#endif
