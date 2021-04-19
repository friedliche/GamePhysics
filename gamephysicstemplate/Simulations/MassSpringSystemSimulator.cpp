#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_gravityForce = Vec3(0.f, -9.81f, 0.f);
	setIntegrator(EULER);
}

const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return "Simple,Complex";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;

	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", this->getIntegratorStr());

	switch (m_iTestCase)
	{
	case 0: 
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		break;
	case 1:
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=10");
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=5");
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0:
		this->DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 50.0f, 0.6*Vec3(1 ,0, 0));
		break;
	case 1:
		this->DUC->setUpLighting(Vec3(), Vec3(1, 0, 0), 5.0f, Vec3(1, 0.5f, 0.65f));
		break;
	default:
		cout << "Shouldn't be here";
		break;
	}

	// draw mass points and springs
	std::for_each(std::begin(this->m_massPoints), std::end(this->m_massPoints), [this](MassPoint &massPoint) {
		this->DUC->drawSphere(massPoint.position, this->m_sphereSize);
	});

	std::for_each(std::begin(this->m_springs), std::end(this->m_springs), [this](Spring &spring) {
		Vec3 p1 = this->m_massPoints[spring.point1].position;
		Vec3 p2 = this->m_massPoints[spring.point2].position;
		
		this->DUC->beginLine();
		this->DUC->drawLine(p1, this->m_springColor1, p2, this->m_springColor2);
		this->DUC->endLine();
	});
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	switch (testCase) {
	case 0:
		cout << "Simple Scenario\n";
		simpleSimulationSetup();
		break;
	case 1:
		cout << "Complex Scenario\n";
		complexSimulationSetup();
		break;
	default:
		cout << "Empty Scenario\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		float inputScale = 0.006f;
		inputWorld = inputWorld * inputScale;

		std::for_each(std::begin(this->m_massPoints), std::end(this->m_massPoints), [inputWorld](MassPoint& massPoint) {
			if (!massPoint.isFixed)
				massPoint.position = massPoint.positionFin + inputWorld;
		});
	}
	else {
		std::for_each(std::begin(this->m_massPoints), std::end(this->m_massPoints), [](MassPoint& massPoint) {
			if (!massPoint.isFixed)
				massPoint.positionFin = massPoint.position;
		});
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	//update current setup for each frame
	switch (m_iIntegrator) {
	case EULER:
		integrateEuler(timeStep);
		break;
	case MIDPOINT:
		//integrateMidpoint(timeStep);
		break;
	case LEAPFROG:
		//integrateLeapFrog(timeStep);
		break;
	default:
		cout << "shouldn't be here";
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass)
{
	this->m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	this->m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	this->m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	MassPoint newMassPoint;
	newMassPoint.position = position;
	newMassPoint.positionFin = Vec3();
	newMassPoint.velocity = velocity;
	newMassPoint.isFixed = isFixed;
	newMassPoint.mass = m_fMass;
	newMassPoint.damping = m_fDamping;
	this->m_massPoints.push_back(newMassPoint);
	return m_iNumberOfMassPoints++;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring newSpring;
	newSpring.point1 = masspoint1;
	newSpring.point2 = masspoint2;
	newSpring.initialLength = initialLength;
	newSpring.stiffness = m_fStiffness;
	++m_iNumberOfSprings;
	m_springs.push_back(newSpring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return this->m_iNumberOfMassPoints;
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return this->m_iNumberOfSprings;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_massPoints[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_massPoints[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	std::for_each(std::begin(this->m_massPoints), std::end(this->m_massPoints), [force](MassPoint& massPoint) {
		massPoint.force = force;
	});
}

const char * MassSpringSystemSimulator::getIntegratorStr()
{
	return "Euler,Midpoint,Leapfrog";
}

void MassSpringSystemSimulator::integrateEuler(float timeStep)
{
	applyExternalForce(m_gravityForce);

	std::for_each(std::begin(this->m_springs), std::end(this->m_springs), [this, timeStep](Spring& spring) {

		Vec3 pos1 = this->m_massPoints[spring.point1].position;
		Vec3 pos2 = this->m_massPoints[spring.point2].position;

		std::vector<Vec3> intForces = spring.computeElasticForces(pos1, pos2);
		
		this->m_massPoints[spring.point1].force -= intForces[0];
		this->m_massPoints[spring.point2].force -= intForces[1];
	});


	std::for_each(std::begin(this->m_massPoints), std::end(this->m_massPoints), [this, timeStep](MassPoint& massPoint) {
		
		// apply damping
		massPoint.force -= this->m_fDamping * massPoint.velocity;

		if (!massPoint.isFixed) {
			massPoint.integratePosition(timeStep);
			massPoint.integrateVelocity(timeStep);
		}

	});
}

void MassSpringSystemSimulator::simpleSimulationSetup()
{
	this->resetScene();

	setMass(10);
	setStiffness(40);
	setDampingFactor(0);

	int i1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int i2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), true);

	addSpring(i1, i2, 1);

	this->m_sphereSize = 0.05*Vec3(1, 1, 1);
	this->m_springColor1 = Vec3(0, 1, 0);
	this->m_springColor2 = Vec3(0, 0, 1);
}

void MassSpringSystemSimulator::complexSimulationSetup()
{
	this->resetScene();

	setMass(10);
	setStiffness(40);
	setDampingFactor(0);

	this->m_sphereSize = 0.02*Vec3(1, 1, 1);
	this->m_springColor1 = Vec3(1, 1, 0);
	this->m_springColor2 = Vec3(1, 0, 1);
}

void MassSpringSystemSimulator::resetScene()
{
	this->m_massPoints.clear();
	this->m_springs.clear();

	m_iNumberOfMassPoints = m_iNumberOfSprings = 0;
}
