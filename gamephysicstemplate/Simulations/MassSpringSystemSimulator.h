#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <vector>
#include <iterator>

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

struct MassPoint {
	Vec3 position, velocity, force;
	Vec3 positionFin;
	float mass, damping;
	bool isFixed;

	void integratePosition(float timeStep) {
		position += timeStep * velocity;
	}
	void integrateVelocity(float timeStep) {
		velocity += timeStep * force/mass;
	}
};

struct Spring {
	int point1, point2;
	float stiffness, initialLength;

	// computes internal force for point1 and point2 resulting from this spring
	std::vector<Vec3> computeElasticForces(Vec3 posPoint1, Vec3 posPoint2) {
		float currentLength = norm(posPoint2 - posPoint1);
		float tmp = stiffness * (currentLength - initialLength);

		std::vector<Vec3> intForces;
		intForces.push_back(tmp * ((posPoint1 - posPoint2) / currentLength));
		intForces.push_back(tmp * ((posPoint2 - posPoint1) / currentLength));

		return intForces;
	}
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	void applyInternalForce(float timeStep, std::vector<MassPoint>& massPoints);
	void applyDamping(MassPoint & massPoint);
	
	// Integration methods
	const char * getIntegratorStr();
	void integrateEuler(float timeStep);
	void integrateMidpoint(float timeStep);
	void integrateLeapFrog(float timeStep);

	// Demo Scenes
	// two mass points connected through a spring
	void simpleSimulationSetup();
	void complexSimulationSetup();
	void resetScene();

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:

	int m_iTestCase;
	int m_iNumberOfMassPoints;
	int m_iNumberOfSprings;
	std::vector<MassPoint> m_massPoints;
	std::vector<Spring> m_springs;

	Vec3 m_sphereSize;
	Vec3 m_springColor1;
	Vec3 m_springColor2;

	Vec3 m_gravityForce;

	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	bool m_drawMSS;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif