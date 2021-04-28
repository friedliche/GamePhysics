#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_pRigidBodySystem = new RigidBodySystem();
	m_iTestCase = 0;

	first = second = true;
}

const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;

}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	int numTemp = m_pRigidBodySystem->getNumRigidBodies();

	DUC->setUpLighting(Vec3(0, 0, 0), Vec3(1, 1, 1), 0.2f, 0.5f*Vec3(1, 1, 1));
	for (int i = 0; i < numTemp; i++) {
		DUC->drawRigidBody(m_pRigidBodySystem->calcTransformMatrixOf(i));
	}
}


void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	switch (testCase) {
	case 0: 
		cout << "demo 1!\n";
		setupDemo1();
		break;
	default:
		cout << "default\n";
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{	
	std::vector<Rigidbody> temp = m_pRigidBodySystem->getRigidBodySystem();
	Vec3 tempTotalTorque;
	Vec3 tempTotalForce;
	int tempTorqueNum = 0;

	for (int i = 0; i < m_pRigidBodySystem->getNumRigidBodies(); i++) {

		tempTotalTorque = Vec3(.0f);
		tempTotalForce = Vec3(.0f);
		tempTorqueNum = temp[i].m_pointsTorque.size();

		for (int j = 0; j < tempTorqueNum; j++) {
			Vec3 xi = (temp[i].m_pointsTorque[j].xi - temp[i].m_boxCenter);
			tempTotalTorque += cross(xi, temp[i].m_pointsTorque[j].fi);
			tempTotalForce += temp[i].m_pointsTorque[j].fi;
		}

		//set total Torque
		m_pRigidBodySystem->setTotalTorque(i, tempTotalTorque);
		//set total Force
		m_pRigidBodySystem->setTotalForce(i, tempTotalForce);
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	//euler integration at page 25
	std::vector<Rigidbody> temp = m_pRigidBodySystem->getRigidBodySystem();

	int num = m_pRigidBodySystem->getNumRigidBodies();

	for (int i = 0; i < num; i++) {
		//x
		m_pRigidBodySystem->setCentralOfMassPosition(i, (temp[i].m_boxCenter + timeStep * temp[i].m_velocity));
		//v linear velocity
		setVelocityOf(i, (temp[i].m_velocity + timeStep * temp[i].m_force / (m_pRigidBodySystem->getTotalMass())));
		//r
		Quat orient = temp[i].m_orientation + (timeStep)* Quat(temp[i].m_angularVelocity.x, temp[i].m_angularVelocity.y, temp[i].m_angularVelocity.z, 1.0f) * temp[i].m_orientation;
		setOrientationOf(i, (orient.unit()));
		//w angular velocity		
		m_pRigidBodySystem->setAngularVelocity(i, temp[i].m_angularVelocity + timeStep * temp[i].m_torque);

		if (first) {
			cout << "------------------------demo1 case------------------------\n";
			cout << "linear and angular velocity of the body: " << getAngularVelocityOfRigidBody(0) << ", " << getLinearVelocityOfRigidBody(0) << "\n";

			Vec3 point = temp[i].m_boxCenter + m_pRigidBodySystem->getRotMatOf(0).transformVector(Vec3(0.3f, 0.5f, 0.25f));
			Vec3 vel = temp[i].m_velocity + cross(temp[i].m_angularVelocity, Vec3(0.3f, 0.5f, 0.25f));

			cout << "world space velocity of point (0.3 0.5 0.25) in world space (" << point << "): " << vel << "\n\n";
			first = false;
		}
		else {
			if (!first && second) {
				cout << "linear and angular velocity of the body: " << getAngularVelocityOfRigidBody(0) << ", " << getLinearVelocityOfRigidBody(0) << "\n";

				Vec3 point = temp[i].m_boxCenter + m_pRigidBodySystem->getRotMatOf(0).transformVector(Vec3(0.3f, 0.5f, 0.25f));
				Vec3 vel = temp[i].m_velocity + cross(temp[i].m_angularVelocity, Vec3(0.3f, 0.5f, 0.25f));



				cout << "world space velocity of point (0.3 0.5 0.25) in world space (" << point << "): " << vel << "\n";
				second = false;
			}
		}
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_pRigidBodySystem->getNumRigidBodies();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	Vec3 xi = m_pRigidBodySystem->getXiOf(i, 0) - m_pRigidBodySystem->getRigidBodySystem()[i].m_boxCenter;
	return m_pRigidBodySystem->getRigidBodySystem()[i].m_boxCenter + m_pRigidBodySystem->getRotMatOf(i).transformVector(xi);
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	Vec3 xi = m_pRigidBodySystem->getXiOf(i, 0) - m_pRigidBodySystem->getRigidBodySystem()[i].m_boxCenter;
	return m_pRigidBodySystem->getRigidBodySystem()[i].m_boxCenter + m_pRigidBodySystem->getRotMatOf(i).transformVector(xi);
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->getRigidBodySystem()[i].m_angularMomentum;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	m_pRigidBodySystem->addRigidBody(position, size, mass);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_pRigidBodySystem->setRotation(i, orientation);
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_pRigidBodySystem->setCentralOfMassVelocity(i, velocity);
}

void RigidBodySystemSimulator::setupDemo1()
{
	this->m_pRigidBodySystem->reset();

	addRigidBody(Vec3(.0f, .0f, .0f), Vec3(1.0f, 0.6f, 0.5f), 2);
	setOrientationOf(0, Quat(0, 0, M_PI_2));
}
