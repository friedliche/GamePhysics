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
}

const char * SPHSystemSimulator::getTestCasesStr()
{
	return nullptr;
}

void SPHSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
}

void SPHSystemSimulator::reset()
{
}

void SPHSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
}

void SPHSystemSimulator::notifyCaseChanged(int testCase)
{
}

void SPHSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void SPHSystemSimulator::simulateTimestep(float timeStep)
{
}

void SPHSystemSimulator::onClick(int x, int y)
{
}

void SPHSystemSimulator::onMouse(int x, int y)
{
}
