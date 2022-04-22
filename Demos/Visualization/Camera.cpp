#include "Camera.h"

using namespace std;

Camera* Camera::m_current = nullptr;
function<void(Real, Real, Real)> Camera::moveFunc = nullptr;
int Camera::m_controlledObject = -1;

Camera* Camera::getCurrent() {
    if (m_current == nullptr) {
        m_current = new Camera();
    }
    return m_current;
}

void Camera::setMoveFunc(function<void(Real, Real, Real)> func) {
    moveFunc = func;
}

void Camera::setControlledObject(int object) {
    m_controlledObject = object;
}

// Follow the character
void Camera::update() {
    //moveFunc(0.1, 0, 0);
}
