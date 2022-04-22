#include "ForceController.h"

using namespace std;

ForceController* ForceController::current = nullptr;

ForceController* ForceController::getCurrent() {
    if (current == nullptr) {
        current = new ForceController();
    }
    return current;
}

void ForceController::setCurrent(ForceController* controller) {
    current = controller;
}

void ForceController::setControlledObject (int index) {
    CONTROLLED_OBJECT = index;
}

// Set the rigidbody's acceleration according to the force being applied on it
void ForceController::setExternalForceAcceleration(SimulationModel &model) {
    
    SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

    // Try find the character
    if (CONTROLLED_OBJECT < 0 || CONTROLLED_OBJECT >= rb.size()) {
        return;
    }
    Vector3r &acc = rb[CONTROLLED_OBJECT]->getAcceleration();

    switch (m_xDirection) {
        case -1:
            acc += Vector3r(-6.0, 0.0, 0.0);
            break;
        case 1:
            acc += Vector3r(6.0, 0.0, 0.0);
            break;
    }

    if (m_jumpPressed) {
        acc += Vector3r(0.0, 1000.0, 0.0);
        m_jumpPressed = false;
    }

    //cout << "Object " << CONTROLLED_OBJECT << " acceleration set to -1." << endl;
}

bool ForceController::keyboardInput(int key, int scancode, int action, int mod) {
    unsigned char c = key;
    if (action == 0) 
        return current->keyReleased(c);
    else if (action == 1) 
        return current->keyPressed(c);
    return false;
}

bool ForceController::keyPressed(unsigned char key) {
    switch (key) {
        case 'A':
            m_leftPressed = true;
            m_xDirection = -1;
            break;
        case 'D':
            m_rightPressed = true;
            m_xDirection = 1;
            break;
        case 'W':
            m_jumpPressed = true;
            break;
        default:
            return false;
    }
    return true;
}

bool ForceController::keyReleased(unsigned char key) {
    switch (key) {
        case 'A':
            m_leftPressed = false;
            if (m_rightPressed) 
                m_xDirection = 1;
            else
                m_xDirection = 0;
            break;
        case 'D':
            m_rightPressed = false;
            if (m_leftPressed)
                m_xDirection = -1;
            else
                m_xDirection = 0;
            break;
        case 'W':
            m_jumpPressed = false;
            break;
        default:
            return false;
    }
    return true;
}

