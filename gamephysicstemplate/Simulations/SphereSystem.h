#pragma once
#include "Simulator.h"

//system of spheres
//every sphere is definied by.. 

struct Sphere {
	Vec3 position, velocity, force;

	void integratePosition(float timeStep) {
		position += timeStep * velocity;
	}
	void integrateVelocity(float timeStep, float mass) {
		velocity += timeStep * force / mass;
	}
};

class SphereSystem {

public:
	//Constructors
	SphereSystem();
	//Functions
	std::vector<Sphere> getSpheres();
	//add new sphere in next level
	void addSphereToSystem();

private:
	//Attributes
	std::vector<Sphere> spheres; // all spheres in a system

	//scaling factor (on y-axis)
	int ylevel = 5;
	//scaling factor (on y-axis)
	int xLevel = 5;
	int zLevel = 5;
};