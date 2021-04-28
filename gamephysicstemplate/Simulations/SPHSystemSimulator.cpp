#include "SPHSystemSimulator.h"

std::function<float(float)> SPHSystemSimulator::m_Kernels[5] = {
	[](float x) {return 1.0f; },              // Constant, m_iKernel = 0
	[](float x) {return 1.0f - x; },          // Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
	[](float x) {return (1.0f - x)*(1.0f - x); }, // Quadratic, m_iKernel = 2
	[](float x) {return 1.0f / (x)-1.0f; },     // Weak Electric Charge, m_iKernel = 3
	[](float x) {return 1.0f / (x*x) - 1.0f; },   // Electric Charge, m_iKernel = 4
};

// SphereSystemSimulator member functions

SPHSystemSimulator::SPHSystemSimulator()
{
	m_iTestCase = 0; // start with demo1
	m_iAccelerator = NAIVEACC;
	m_fMass = 10.0f;
	m_fRadius = 0.05f;
	m_fForceScaling = 0.0f;
	m_fDamping = 5.0f;
	m_pSphereSystem = new SphereSystem();
	m_iKernel = 0;
	m_iIntegrator = MIDPOINT; //0 midpoint, 1 leap frog
}

const char * SPHSystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2, Demo 3";
}

const char * SPHSystemSimulator::getIntegCasesStr()
{
	return "Midpoint,Leapfrog";
}

void SPHSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;

	TwType TW_TYPE_INTEGCASE = TwDefineEnumFromString("Integration", getIntegCasesStr());
	switch (m_iTestCase)
	{
	case 0:
		TwAddVarRW(DUC->g_pTweakBar, "Integration", TW_TYPE_INTEGCASE, &m_iIntegrator, "");
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=5");
		TwAddVarRW(DUC->g_pTweakBar, "Number", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Radius", TW_TYPE_FLOAT, &m_fRadius, "min=0.04, step=0.001");
		break;
	case 1:
		break;
	case 2:
		break;
	default:break;
	}
}

void SPHSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), Vec3(1, 1, 0), 2000.0f, Vec3(1, 0.5f, 0.65f));
	std::vector<Sphere> tmpSpheres = m_pSphereSystem->getSpheres();

	// draw spheres
	for (int i = 0; i < tmpSpheres.size(); i++) {
		DUC->drawSphere(tmpSpheres[i].position, m_fRadius* Vec3(1, 1, 1));
		//TODO: draw grid 
	}

	switch (m_iTestCase) {
	case 1:
		break;
	case 0:
		break;
	case 2:
		//different colours and size of spheres
		break;
	default:
		break;
	}
}

void SPHSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void SPHSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1 !\n";
		setupDemo1();
		break;
	case 1:
		cout << "Demo 2 !\n";

		break;
	case 2:
		cout << "Demo 3 !\n";

		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void SPHSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	//gravity
	externalForce = Vec3(0, -9.81f, 0);
}

void SPHSystemSimulator::applyExternalForce(Vec3 force)
{
	std::vector<Sphere> tmpSpheres = m_pSphereSystem->getSpheres();

	for (int i = 0; i < tmpSpheres.size(); ++i) {
		tmpSpheres[i].force = force;
	}
}

void SPHSystemSimulator::resetScene()
{
	this->m_pSphereSystem->clearScene();
}

void SPHSystemSimulator::setupDemo1()
{
	resetScene();

	Sphere s1, s2;
	s1.position = Vec3(0.0f);
	s1.velocity = Vec3(1.0f, 1.0f, 0.0f);
	s1.force = Vec3(0.0f);
	this->m_pSphereSystem->addSphere(s1);

	s2.position = Vec3(0.0f, -0.5f, 0.0f);
	s2.velocity = Vec3(0.0f, 0.0f, 0.0f);
	s2.force = Vec3(0.0f);
	this->m_pSphereSystem->addSphere(s2);
	
	m_iNumSpheres = 2;
}

//repulsion forces
void SPHSystemSimulator::integrateMidpoint(float timeStep)
{
	std::vector<Sphere>& tmpSpheres = m_pSphereSystem->getSpheres();

	for (int i = 0; i < tmpSpheres.size(); ++i) {
		Sphere sphere = tmpSpheres[i];
		
		sphere.integratePosition(timeStep / 2); //xTilde		
		sphere.integrateVelocity(timeStep / 2, this->m_fMass); //velocity at xTilde
				
		m_pSphereSystem->getSpheres()[i].position += timeStep * sphere.velocity; //new position
	}

	applyExternalForce(externalForce); //clear forces

	//new velocity
	for (int i = 0; i < tmpSpheres.size(); ++i) {
		Sphere sphere = tmpSpheres[i];

		handleBoundariesHits(this->m_pSphereSystem->getSpheres()[i]);
		this->m_pSphereSystem->getSpheres()[i].velocity += timeStep * sphere.force / this->m_fMass;
	}
}

void SPHSystemSimulator::integrateLeapFrog(float timeStep)
{
	std::vector<Sphere> sphs = m_pSphereSystem->getSpheres();

	std::for_each(std::begin(sphs), std::end(sphs), [this, timeStep](Sphere& sphere) {
		sphere.integrateVelocity(timeStep, this->m_fMass);
		sphere.integratePosition(timeStep);
	});
}



void SPHSystemSimulator::simulateTimestep(float timeStep)
{
	//wenn geadded, adden
	if (m_iNumSpheres != m_pSphereSystem->getSpheres().size()) {
		m_pSphereSystem->addSphereToSystem();
	}

	std::vector<Sphere> tmp = m_pSphereSystem->getSpheres();
	int a = 0;

	applyExternalForce(externalForce);

	switch (m_iIntegrator) {
	case MIDPOINT:
		integrateMidpoint(timeStep);
		break;
	case LEAPFROG:
		integrateLeapFrog(timeStep);
		break;
	default:
		cout << "shouldn't be here";
		break;
	}
	
	//TODO: check for collisions
	for (int i = 0; i < m_iNumSpheres; i++) {
		for (int j = 0; j < m_iNumSpheres; j++) {
			float posDif = norm(tmp[i].position - tmp[j].position);
			float radQuad = m_fRadius + m_fRadius;

			//collision, naiv approach
			//compute force for every sphere according to f(d)
			if (posDif <= radQuad) {
				tmp[i].force = m_Kernels[m_iKernel]((posDif / 2 * radQuad));
			}
		}
		a++;
	}
}

void SPHSystemSimulator::handleBoundariesHits(Sphere  sphere)
{
	if (sphere.position.y <= -0.5f) {
		sphere.position.y = -0.5f;
		sphere.velocity.y *= -0.6f;
	}

	if (sphere.position.x >= 0.5f || sphere.position.x <= -0.5f) {
		sphere.position.x = (sphere.position.x >= 0.5f ? 0.5f : -0.5f);
		sphere.velocity.x *= -0.6f;
	}
}

void SPHSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void SPHSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void SPHSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void SPHSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

void SPHSystemSimulator::setRadius(float radius)
{
	m_fRadius = radius;
}

int SPHSystemSimulator::getNumberOfSpheres()
{
	return m_iNumSpheres;
}