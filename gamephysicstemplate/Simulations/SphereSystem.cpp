#include "SphereSystem.h"

SphereSystem::SphereSystem()
{


}

std::vector<Sphere>& SphereSystem::getSpheres()
{
	return spheres;
}

void SphereSystem::addSphereToSystem()
{
	Sphere newSphere;
	newSphere.position = Vec3(0.1f * xLevel, 0.1f * ylevel, 0.1f * zLevel);
	newSphere.velocity = Vec3();
	newSphere.force = Vec3();
	xLevel--;

	if (zLevel <= -5 && xLevel < -5) {
		ylevel--; zLevel = xLevel = 5;
	}

	if (xLevel < -5) {
		zLevel--; xLevel = 5;
	}
	spheres.push_back(newSphere);
}

int SphereSystem::addSphere(Sphere sph)
{
	spheres.push_back(sph);
	return spheres.size();
}

void SphereSystem::clearScene()
{
	spheres.clear();
}
