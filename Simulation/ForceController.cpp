#include "ForceController.h"

using namespace std;

int ForceController::CONTROLLED_OBJECT = -1;
int ForceController::forceDirection = 0;

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

    switch (forceDirection) {
        case -1:
            acc += Vector3r(-6.0, 0.0, 0.0);
            break;
        case 1:
            acc += Vector3r(6.0, 0.0, 0.0);
            break;
    }

    //cout << "Object " << CONTROLLED_OBJECT << " acceleration set to -1." << endl;
}

void ForceController::applyForceLeft() {
    forceDirection = -1;
    cout << "Moving left." << endl;
}

void ForceController::applyForceRight() {
    forceDirection = 1;
    cout << "Moving right." << endl;
}
