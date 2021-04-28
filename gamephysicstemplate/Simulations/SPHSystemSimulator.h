#ifndef SPHSYSTEMSIMULATOR_h
#define SPHSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "SphereSystem.h"
#include <vector>
#include <iterator>

#define NAIVEACC 0
#define GRIDACC 1

//for integration
#define MIDPOINT 0
#define LEAPFROG 1

class SPHSystemSimulator :public Simulator {
public:
	// Construtors
	SPHSystemSimulator();
	// Functions
	const char * getTestCasesStr();
	const char * getIntegCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void integrateMidpoint(float timeStep);
	void integrateLeapFrog(float timeStep);
	void simulateTimestep(float timeStep);
	void handleBoundariesHits(Sphere  sphere);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	void setMass(float mass);
	void setDampingFactor(float damping);
	void setRadius(float radius);
	int getNumberOfSpheres();

	void applyExternalForce(Vec3 force);

	void resetScene();
	void setupDemo1();

protected:
	// Attributes
	Vec3 externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	//mass, radius is identical
	float m_fMass;
	float m_fRadius;
	float m_fForceScaling;
	float m_fDamping;
	int   m_iNumSpheres;

	int   m_iKernel; // index of the m_Kernels[5], more detials in SphereSystemSimulator.cpp
	static std::function<float(float)> m_Kernels[5];

	int   m_iAccelerator; // switch between NAIVEACC and GRIDACC, (optionally, KDTREEACC, 2)
	int   m_iIntegrator; //I added this, so you can switch between leap frog and midpoint (like in massspringsystem)

	SphereSystem * m_pSphereSystem; // add your own sphere system member!
	// for Demo 3 only:
	// you will need multiple SphereSystem objects to do comparisons in Demo 3
	// m_iAccelerator should be ignored.
	// SphereSystem * m_pSphereSystemGrid; 

};

#endif
