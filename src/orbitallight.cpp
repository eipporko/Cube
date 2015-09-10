#include "orbitallight.h"

vector<OrbitalLight*> OrbitalLight::lights;
float OrbitalLight::lightPosition[MAX_LIGHTS*3];
float OrbitalLight::lightColor[MAX_LIGHTS*3];
float OrbitalLight::lightIntensity[MAX_LIGHTS];
int OrbitalLight::lightCount;

void OrbitalLight::update(){
    this->position = glm::rotate(this->position, glm::radians(this->stepRotation), this->axisRotation );
};

void OrbitalLight::pushToGPU()  {
    vector<OrbitalLight *> list;
    list.push_back(this);
    OrbitalLight::pushToGPU(list);
};