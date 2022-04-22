#ifndef __FORCECONTROLLER_h__
#define __FORCECONTROLLER_h__

#include "Common/Common.h"
#include "SimulationModel.h"
#include "RigidBody.h"

using namespace PBD;

// Monitors the amount of external force currently applied on an object
//  Force might be applied through some external input
class ForceController {
    private:
        static ForceController* current;
        Vector3r m_force;
        int m_xDirection = 0;   // -1 if we're going left, 1 if right
        bool m_leftPressed = 0; // True if left key is held down in latest frame
        bool m_rightPressed = 0;
        bool m_jumpPressed = 0;

    public:
        int CONTROLLED_OBJECT = -1;   // Object to be moved by external force (index in rigidbody list)

    public:
        // Singleton
        static ForceController* getCurrent();
        static void setCurrent(ForceController* controller);

        void setControlledObject (int index);

        // Set the rigidbody's acceleration according to the force being applied on it
        virtual void setExternalForceAcceleration(SimulationModel &model);

        // OpenGL callbacks
        // Called when there's any input. True if the function wants to 'use up' this event
        static bool keyboardInput(int key, int scancode, int action, int mod);
        bool keyPressed(unsigned char key);
        bool keyReleased(unsigned char key);
};

/*
    public:
        unsigned int getControlledObject() { return m_object}; // Gets the index of the object the force is acting on
        Vector3r getCurrentForce() { return m_force; }; // Gets the force being applied on the object
        setCurrentForce(Vector3r force) { m_force = force };    // FIX MEMORY ALLOCATION!!!
};
*/

#endif