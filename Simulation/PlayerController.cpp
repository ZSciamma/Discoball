#include "PlayerController.h"

#include "Thrusters.h"
#include "Cannon.h"

PlayerController* PlayerController::current = nullptr;
PlayerController::MousePosFct PlayerController::mousePosFunc = nullptr;
PlayerController::WorldToScreenFct PlayerController::worldToScreenFunc = nullptr;

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

    mousePosFunc(current->oldMouseX, current->oldMouseY);

    current->m_mousePressed = true;

    return false;  // Pretend we don't want it so others can use it
}

// Called by the physics loop once it's ready for us to add recoil
//  Since the caller provides the SimulationModel, we can't do these
//  calculations until now (actually this is untrue because we could pass the model in at the start)
// Maybe we should change this in the future, then.
void PlayerController::applyRecoil(SimulationModel &model) {
    if (!m_mousePressed) {
        return;
    }
    m_mousePressed = false;
    
    Vector3r bulletPos;
    Vector3r force = calculateRecoil(model, bulletPos);

    Thrusters::getCurrent()->applyPropulsion(model, force);
    Cannon::shootBullet(model, bulletPos, -force);
}

Vector3r PlayerController::calculateRecoil(SimulationModel &model, Vector3r &bulletPos) {
    SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

    // Try find the character
    if (m_playerObj < 0 || m_playerObj >= rb.size()) {
        return Vector3r(0, 0, 0);
    }

    // This might not be accurate. Maybe the callback interrupted some physics,
    //  and this is not what the player sees because it's moved since
    Vector3r playerWorldPos = rb[m_playerObj]->getPosition();                      
    double playerScreenPosX, playerScreenPosY;

    worldToScreenFunc(playerWorldPos, playerScreenPosX, playerScreenPosY);

    // Calculate angle between mouse and character
    double xDist = oldMouseX - playerScreenPosX;
    double yDist = oldMouseY - playerScreenPosY;
    if (xDist == 0)
        xDist = 0.1;
    double angle = atan(yDist / xDist);

    // Calculate force in each direction
    double xFactor = cos(angle);
    if (xDist > 0)
        xFactor = -xFactor;
    double yFactor = -sin(angle);
    if (xDist > 0)
        yFactor = -yFactor;
    double magnitude = 1000;
    Vector3r force = Vector3r(magnitude*xFactor, magnitude*yFactor, 0);


    // Calculate bullet position
    double worldDist = 1;
    Vector3r bulletLocVector = worldDist * Vector3r(-xFactor, -yFactor, 0);
    Vector3r bulletWorldPos = playerWorldPos + bulletLocVector;

    bulletPos.x() = bulletWorldPos.x();
    bulletPos.y() = bulletWorldPos.y();
    bulletPos.z() = bulletWorldPos.z();

    return force;
}
