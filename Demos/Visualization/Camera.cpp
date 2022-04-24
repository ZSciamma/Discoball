#include "Camera.h"
#include "MiniGL.h"

#include <cmath>

using namespace PBD;
using namespace std;

double Camera::moveThreshold = 1.5;
double Camera::maxMoveDist = 0.4;   // Max world dist camera will move in a frame
double Camera::minMoveDist = 1.0;   // Min distance camera will bother to move for

Camera* Camera::current = nullptr;
int Camera::playerObj = -1;

Camera* Camera::getCurrent() {
    if (current == nullptr) {
        current = new Camera();
    }
    return current;
}

void Camera::init(SimulationModel *model, int playerObject) {
    playerObj = playerObject;
    m_model = model;

    m_screenW = MiniGL::getWidth();
    m_screenH = MiniGL::getHeight();
}

// Follow the character
// Some calculations could possibly be simpler if we compared the lookAt point to the cube's position
void Camera::update() {
    // Get character's screen position
    SimulationModel::RigidBodyVector &rb = m_model->getRigidBodies();
    Vector3r playerWorldPos = rb[playerObj]->getPosition();                      // Probably not super accurate but doesn't matter
    double playerScreenPosX, playerScreenPosY;
    MiniGL::project(playerWorldPos, playerScreenPosX, playerScreenPosY);

    // Check how much character has moved
    double xDisp = playerScreenPosX - m_oldPlayerScreenPosX;
    double yDisp = playerScreenPosY - m_oldPlayerScreenPosY;

    m_oldPlayerScreenPosX = playerScreenPosX;
    m_oldPlayerScreenPosY = playerScreenPosY;

    if (isFirstUpdate) {
        m_xMoveDir = ((m_oldPlayerScreenPosX - (double) m_screenW/2) > 0) * 2 - 1;
        m_yMoveDir = ((-m_oldPlayerScreenPosY + (double) m_screenH/2) > 0) * 2 - 1;
        //cout << "Start screen pos: " << m_oldPlayerScreenPosX << ", " << m_oldPlayerScreenPosY << endl;
        //cout << "Start move dir: " << m_xMoveDir << ", " << m_yMoveDir << endl;
        isFirstUpdate = false;
        return;
    }

    //cout << "Character moved " << xDisp << ", " << yDisp << " in screen coords" << endl;

    // Check if we should stop or start moving
    if (m_currentlyMoving) {
        // Stop moving if player has changed direction (in x or y axis)
        bool shouldStopMoving = (xDisp>0 && m_xMoveDir<0) | (xDisp<0 && m_xMoveDir>0) || (yDisp>0 && m_yMoveDir<0) | (yDisp<0 && m_yMoveDir>0);
        if (shouldStopMoving) {
            m_currentlyMoving = false;
            m_freezePosition = Vector2r(playerScreenPosX, playerScreenPosY);
        }
    } else {
        // Start moving if we've moved a certain threshold since last time we stopped
        double newDist = sqrt(pow(playerScreenPosX - m_freezePosition.x(), 2) + pow(playerScreenPosY - m_freezePosition.y(), 2));
        if (newDist > moveThreshold) {
            m_currentlyMoving = true;
        }
    }

    //if (!m_currentlyMoving)
    //    return;
    
    // Move camera
    // Check how far player is from center of screen
    double distX = playerScreenPosX - (double) m_screenW / 2.0;
    double distY = -playerScreenPosY + (double) m_screenH / 2.0; // Flip because origin is top left in screen coords

    double screenMoveX = distX;
    double screenMoveY = distY;

    //cout << "Player position: " << playerScreenPosX << ", " << playerScreenPosY << endl;
    //cout << "Planned move: " << screenMoveX << ", " << screenMoveY << endl;

    double screenToWorldFact = 100;
    double worldMoveX = screenMoveX / screenToWorldFact;
    double worldMoveY = screenMoveY / screenToWorldFact;

    double toMove = sqrt(pow(worldMoveX, 2) + pow(worldMoveY, 2)); 

    // Scale down movement magnitude for smooth camera movement over time
    // If it's less than 1.5 away, then don't move
    // If it's 1.5 away, then log(3*x - 3.5) == log(1) = 0.
    // More than 1.5 away, log(3*x - 3.5) gets bigger gradually
    // If it's 4.5 away, log(3*x - 3.5) == log(10) = 1.
    // If it's more than 6.5 away, we cap it at 1
    double magnitude;
    if (toMove <= 1.5) {
        //cout << "Too small for any movement" << endl << endl;
        return;
    } else if (toMove >= 4.5) {
        magnitude = 1;
    } else {
        magnitude = log10(3*toMove-3.5);
    }

    double moveFract = magnitude / toMove;
    worldMoveX *= moveFract;
    worldMoveY *= moveFract;

    //cout << "Movement: " << -worldMoveX << ", " << -worldMoveY << endl << endl;

    // Kind of a hack because we know the camera is always on the x/y plane. If
    //  it weren't, we'd have to do some conversion to world coordnates using
    //  the eye position or something.
    MiniGL::move(-worldMoveX, -worldMoveY, 0.0);
}



   /*
    // Project camera movement vector to world coordinates
    Vector3r worldMove = Vector3r(0, 0, 0);
    cout << "Planned move: " << screenMoveX << ", " << screenMoveY << endl;
    MiniGL::unproject(screenMoveX, screenMoveY, worldMove);
    double toMove = worldMove.norm();
    if (toMove > maxMoveDist) {
        // Restrict magnitude of camera movement this fram
        worldMove *= maxMoveDist / toMove;
    }
    cout << "Camera moving: " << worldMove.x() << ", " << worldMove.y() << ", " << worldMove.z() << endl << endl;

    // 
    double eyeX, eyeY, eyeZ;
    MiniGL::getEyePoint(eyeX, eyeY, eyeZ);
    cout << "Eye Pos: " << eyeX << ", " << eyeY << ", " << eyeZ << endl;


    cout << "Camera moving: " << worldMove.x() << ", " << worldMove.y() << ", " << worldMove.z() << endl << endl;
    //MiniGL::move(worldMove.x(), worldMove.y(), worldMove.z());
    //MiniGL::move(0.1, 0.1, 0.1);
    */

    // If either x or y is contrary to the direction we were going, stop moving
    // If moving, check how far we need to move
        // If player is very close, follow player along
        // If distance to player is too big, just cover a certain fraction of it (such that speed will look good)

