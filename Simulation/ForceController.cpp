#include "ForceController.h"

using namespace std;

ForceController* ForceController::current = nullptr;
ForceController::MousePosFct ForceController::mousePosFunc = nullptr;
ForceController::WorldToScreenFct ForceController::worldToScreenFunc = nullptr;

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
    controlledObject = index;
}

// Set the rigidbody's acceleration according to the force being applied on it
// Called by the physics when it's ready for the force to be applied
void ForceController::setExternalForceAcceleration(SimulationModel &model) {
    
    SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

    // Try find the character
    if (controlledObject < 0 || controlledObject >= rb.size()) {
        return;
    }
    Vector3r &acc = rb[controlledObject]->getAcceleration();

    if (m_mousePressed) {
        m_mousePressed = false;

        Vector3r playerWorldPos = rb[controlledObject]->getPosition();
        double playerScreenPosX, playerScreenPosY;

        worldToScreenFunc(playerWorldPos, playerScreenPosX, playerScreenPosY);
        
        //std::cout << "Mouse position: " << mouse_old_x << ", " << mouse_old_y << std::endl;
        //std::cout << "Player position: " << playerScreenPosX << ", " << playerScreenPosY << std::endl;

        // Calculate angle between mouse and character
        double xDist = mouse_old_x - playerScreenPosX;
        double yDist = mouse_old_y - playerScreenPosY;
        if (xDist == 0)
            xDist = 0.1;
        double angle = atan(yDist / xDist);
        //cout << "angle" << angle << endl;

        // Calculate force in each direction
        double xFactor = cos(angle);
        if (xDist > 0)
            xFactor = -xFactor;
        double yFactor = -sin(angle);
        if (xDist > 0)
            yFactor = -yFactor;
        double magnitude = 1000;
        Vector3r force = Vector3r(magnitude*xFactor, magnitude*yFactor, 0);

        // Apply force
        acc += force;
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
}

bool ForceController::mouseInput(int button, int action, int mods) {
    if (action == 1) 
        return current->mousePressed(button, action, mods);
    return false;
}

bool ForceController::mousePressed(int button, int action, int mods) {
    // Get current mouse position from GL;
    mousePosFunc(mouse_old_x, mouse_old_y);

    // Set m_mousePressed for next time physics comes around
    m_mousePressed = true;

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

