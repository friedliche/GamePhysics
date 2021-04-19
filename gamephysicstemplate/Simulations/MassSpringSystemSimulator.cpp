#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_gravityForce = Vec3(0.f, -30.f, 0.f);
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
	this->m_iTestCase = testCase;
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
		float inputScale = 0.005f;
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
	applyExternalForce(m_gravityForce);
	applyInternalForce(timeStep, this->m_massPoints);

	switch (m_iIntegrator) {
	case EULER:
		integrateEuler(timeStep);
		break;
	case LEAPFROG:
		integrateLeapFrog(timeStep);
		break;
	case MIDPOINT:
		integrateMidpoint(timeStep);
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

void MassSpringSystemSimulator::applyInternalForce(float timeStep, std::vector<MassPoint> &massPoints)
{
	std::for_each(std::begin(this->m_springs), std::end(this->m_springs), [this, timeStep, &massPoints](Spring& spring) {

		Vec3 pos1 = massPoints[spring.point1].position;
		Vec3 pos2 = massPoints[spring.point2].position;

		std::vector<Vec3> intForces = spring.computeElasticForces(pos1, pos2, this->m_fStiffness);
		
		massPoints[spring.point1].force -= intForces[0];
		massPoints[spring.point2].force -= intForces[1];
	});
}

void MassSpringSystemSimulator::applyDamping(MassPoint & massPoint)
{
	massPoint.force -= this->m_fDamping * massPoint.velocity;
}

void MassSpringSystemSimulator::handleBoundariesHits(MassPoint & massPoint)
{
	if (massPoint.position.y <= -0.5f) {
		massPoint.position.y = -0.5f;
		massPoint.velocity.y *= -0.6f;
	}
	
	if (massPoint.position.x >= 0.5f || massPoint.position.x <= -0.5f) {
		massPoint.position.x = (massPoint.position.x >= 0.5f? 0.5f : -0.5f);
		massPoint.velocity.x *= -0.6f;
	}

}

const char * MassSpringSystemSimulator::getIntegratorStr()
{
	return "Euler,Leapfrog,Midpoint";
}

void MassSpringSystemSimulator::integrateEuler(float timeStep)
{
	std::for_each(std::begin(this->m_massPoints), std::end(this->m_massPoints), [this, timeStep](MassPoint& massPoint) {
	
		applyDamping(massPoint);

		if (!massPoint.isFixed) {
			massPoint.integratePosition(timeStep);
			handleBoundariesHits(massPoint);
			massPoint.integrateVelocity(timeStep, this->m_fMass);
		}
	});
}

void MassSpringSystemSimulator::integrateMidpoint(float timeStep)
{
	std::vector<MassPoint> tmpPoints = this->m_massPoints;

	for (int i = 0; i < tmpPoints.size(); ++i) {
		MassPoint massPoint = tmpPoints[i];
		applyDamping(massPoint);

		if (!massPoint.isFixed){
			massPoint.integratePosition(timeStep / 2);
			massPoint.integrateVelocity(timeStep / 2, this->m_fMass);

			this->m_massPoints[i].position += timeStep * tmpPoints[i].velocity;
		}
	}

	applyExternalForce(m_gravityForce);
	applyInternalForce(timeStep, tmpPoints);

	for (int i = 0; i < tmpPoints.size(); ++i) {
		MassPoint massPoint = tmpPoints[i];
		applyDamping(massPoint);

		if (!massPoint.isFixed) {
			handleBoundariesHits(this->m_massPoints[i]);
			this->m_massPoints[i].velocity += timeStep * massPoint.force / this->m_fMass;
		}
	}
}

void MassSpringSystemSimulator::integrateLeapFrog(float timeStep)
{
	std::for_each(std::begin(this->m_massPoints), std::end(this->m_massPoints), [this, timeStep](MassPoint& massPoint) {

		applyDamping(massPoint);

		if (!massPoint.isFixed) {
			massPoint.integrateVelocity(timeStep, this->m_fMass);
			massPoint.integratePosition(timeStep);
			handleBoundariesHits(massPoint);
		}
	});
}

void MassSpringSystemSimulator::simpleSimulationSetup()
{
	this->resetScene();

	setMass(10.f);
	setStiffness(40.f);
	setDampingFactor(0.f);

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

	setMass(10.f);
	setStiffness(150.f);
	setDampingFactor(5.f);

	//first option: one mass and one spring
	int p1 = addMassPoint(Vec3(-0.25f, 0.25f, 0), Vec3(0, 0, 0), FALSE);
	int p2 = addMassPoint(Vec3(-0.25f, 0.5f, 0), Vec3(0.7f, 0, 0), TRUE);
	//the first and second point in m_points
	addSpring(p1, p2, 0.1f);

	//second option: two masses and two springs
	int p3 = addMassPoint(Vec3(0.3f, 0.498f, 0), Vec3(-0.2, 0, 0), FALSE);
	int p4 = addMassPoint(Vec3(0.3f, 0.499f, 0), Vec3(0.1, 0, 0), FALSE);
	addSpring(p3, p4, 0.1f);
	//"ceil"
	int p5 = addMassPoint(Vec3(0.4f, 0.5f, 0), Vec3(0, 0, 0), TRUE);
	addSpring(p4, p5, 0.1f);

	//third option: a pyramid
	addMassPoint(Vec3(-0.1f, -0.2f, 0), Vec3(), FALSE);
	addMassPoint(Vec3(-0.1f, -0.4f, -0.25f), Vec3(), FALSE);
	addMassPoint(Vec3(0.f, -0.4f, 0.0f), Vec3(), FALSE);
	addMassPoint(Vec3(-0.1f, -0.4f, 0.25f), Vec3(), FALSE);
	addMassPoint(Vec3(-0.2f, -0.4f, 0), Vec3(), FALSE);
	addSpring(5, 6, 0.2f);
	addSpring(5, 7, 0.2f);
	addSpring(5, 8, 0.2f);
	addSpring(5, 9, 0.2f);
	addSpring(6, 7, 0.2f);
	addSpring(6, 8, 0.2f);
	addSpring(6, 9, 0.2f);
	addSpring(7, 8, 0.2f);
	addSpring(7, 9, 0.2f);
	addSpring(8, 9, 0.2f);

	//fourth option: one mass and one spring (vertical)
	addMassPoint(Vec3(0, 0.25f, 0), Vec3(0, 0, 0), FALSE);
	addMassPoint(Vec3(0, 0.5f, 0), Vec3(0.7, 0, 0), TRUE);
	addSpring(10, 11, 0.1f);

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
