#include "Thrusters.h"

#include "RigidBody.h"

using namespace std;

Thrusters* Thrusters::current = nullptr;

Thrusters* Thrusters::getCurrent() {
    if (current == nullptr) {
        current = new Thrusters();
    }
    return current;
}

void Thrusters::setCurrent(Thrusters* controller) {
    current = controller;
}

void Thrusters::setControlledObject (int index) {
    m_controlledObj = index;
}

void Thrusters::applyPropulsion(SimulationModel &model, Vector3r force) {
    // Try find the character
    SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
    if (m_controlledObj < 0 || m_controlledObj >= rb.size()) {
        return;
    }
    Vector3r &acc = rb[m_controlledObj]->getAcceleration();

    // Apply force
    acc += force;
}
