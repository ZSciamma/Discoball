#include "PlayerController.h"

#include "Demos/Visualization/MiniGL.h"
#include "Simulation/Thrusters.h"

PlayerController* PlayerController::current = nullptr;

PlayerController* PlayerController::getCurrent() {
    if (current == nullptr) {
        current = new PlayerController();
    }
    return current;
}

void PlayerController::init(int playerObj) {
    m_playerObj = playerObj;

    Thrusters::getCurrent()->setControlledObject(m_playerObj);
}

bool PlayerController::mousePressed(int button, int action, int mods) {
    if (action != 1) 
        return false;

    MiniGL::getMousePos(current->oldMouseX, current->oldMouseY);

    current->m_mousePressed = true;

    return false;  // Pretend we don't want it so others can use it
}

// Called by the physics loop once it's ready for us to add recoil
//  Since the caller provides the SimulationModel, we can't do these
//  calculations until now.
void PlayerController::applyRecoil(SimulationModel &model) {
    if (!m_mousePressed) {
        return;
    }
    m_mousePressed = false;
    
    Vector3r force = calculateRecoil(model);

    Thrusters::getCurrent()->applyPropulsion(model, force);
}

Vector3r PlayerController::calculateRecoil(SimulationModel &model) {
    SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

    // Try find the character
    if (m_playerObj < 0 || m_playerObj >= rb.size()) {
        return Vector3r(0, 0, 0);
    }

    Vector3r playerWorldPos = rb[m_playerObj]->getPosition();                      // IS THIS ACCURATE? MAYBE THIS IS NOT WHAT THE PLAYER SEES! WHAT IF THE CALLBACK INTERRUPTED SOME PHYSICS, AND IT'S MOVED SINCE?
    double playerScreenPosX, playerScreenPosY;

    MiniGL::project(playerWorldPos, playerScreenPosX, playerScreenPosY);
    //std::cout << "Mouse position: " << oldMouseX << ", " << oldMouseY << std::endl;
    //std::cout << "Player position: " << playerScreenPosX << ", " << playerScreenPosY << std::endl;

    // Calculate angle between mouse and character
    double xDist = oldMouseX - playerScreenPosX;
    double yDist = oldMouseY - playerScreenPosY;
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

    return force;
}
