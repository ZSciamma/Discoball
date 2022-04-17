#ifndef __FORCECONTROLLER_h__
#define __FORCECONTROLLER_h__

#include "Common/Common.h"
#include "SimulationModel.h"
#include "RigidBody.h"

using namespace PBD;

// Monitors the amount of external force currently applied on an object
//  Force might be applied through some external input
class ForceController {
    public:
        static int CONTROLLED_OBJECT;   // Object to be moved by external force (index in rigidbody list)
        static Vector3r m_force;
        static int forceDirection;
    public:
        static void setControlledObject (int index);

        // Set the rigidbody's acceleration according to the force being applied on it
        static void setExternalForceAcceleration(SimulationModel &model);

        static void applyForceLeft();
        static void applyForceRight();
};

/*
    public:
        unsigned int getControlledObject() { return m_object}; // Gets the index of the object the force is acting on
        Vector3r getCurrentForce() { return m_force; }; // Gets the force being applied on the object
        setCurrentForce(Vector3r force) { m_force = force };    // FIX MEMORY ALLOCATION!!!
};
*/

#endif