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
        double mouse_old_x; // Mouse position last time we fetched it, which could be a long time ago
        double mouse_old_y;
        Vector3r m_force;
        int m_xDirection = 0;   // -1 if we're going left, 1 if right
        bool m_mousePressed = false;
        bool m_leftPressed = false; // True if left key is held down in latest frame
        bool m_rightPressed = false;
        bool m_jumpPressed = false;

        typedef std::function<void(double&, double&)> MousePosFct;
        static MousePosFct mousePosFunc;

    public:
        int controllerObject = -1;   // Object to be moved by external force (index in rigidbody list)

        // Singleton
        static ForceController* getCurrent();
        static void setCurrent(ForceController* controller);

        void setControlledObject (int index);

        // Set the rigidbody's acceleration according to the force being applied on it
        void setExternalForceAcceleration(SimulationModel &model);

        // OpenGL callbacks
        // Called when there's any input. True if the function wants to 'use up' this event
        // Need to be static so they can be set as callbacks
        static void setMousePosFunc(MousePosFct func) {mousePosFunc = func;}
        static bool mouseInput(int button, int action, int mods);
        static bool keyboardInput(int key, int scancode, int action, int mod);

        bool mousePressed(int button, int action, int mods);
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