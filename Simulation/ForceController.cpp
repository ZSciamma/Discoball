#include "ForceController.h"

using namespace std;

ForceController* ForceController::current = nullptr;
ForceController::MousePosFct ForceController::mousePosFunc = nullptr;

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
    controllerObject = index;
}

// Set the rigidbody's acceleration according to the force being applied on it
// Called by the physics when it's ready for the force to be applied
void ForceController::setExternalForceAcceleration(SimulationModel &model) {
    
    SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

    // Try find the character
    if (controllerObject < 0 || controllerObject >= rb.size()) {
        return;
    }
    Vector3r &acc = rb[controllerObject]->getAcceleration();

    if (m_mousePressed) {
        acc += Vector3r(0.0, 1000.0, 0.0);
        m_mousePressed = false;
    }

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

    //cout << "Object " << controllerObject << " acceleration set to -1." << endl;
}

bool ForceController::mouseInput(int button, int action, int mods) {
    if (action == 1) 
        return current->mousePressed(button, action, mods);
    return false;
}

bool ForceController::mousePressed(int button, int action, int mods) {
    // Get mouse position from GL
    mousePosFunc(mouse_old_x, mouse_old_y);

    // Set m_mousePressed for next time physics comes around
    m_mousePressed = true;

    //cout << "Mouse pressed! Mouse position: " << mouse_old_x << ", " << mouse_old_y << endl;
    return false;       // Pretend we don't want it so others can use it
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

