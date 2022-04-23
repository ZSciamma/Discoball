#ifndef __Thrusters_h__
#define __Thrusters_h__

#include "Common/Common.h"
#include "SimulationModel.h"

using namespace PBD;

// Monitors the amount of external force currently applied on an object
//  Force might be applied through some external input
class Thrusters {
    private:
        static Thrusters* current;                                                  // DOESN'T NEED TO BE A SINGLETON! CAN PROBABLY JUST BE STATIC!!

        int m_controlledObj = -1;
        //bool m_thrustersScheduled = false;

    public:
        // Singleton
        static Thrusters* getCurrent();
        static void setCurrent(Thrusters* controller);

        void setControlledObject (int index);

        // Recoil
        void applyPropulsion(SimulationModel &model, Vector3r force);  // Physics is ready for force to be applied
};


        //int m_xDirection = 0;   // -1 if we're going left, 1 if right

        // double mouse_old_x; // Mouse position last time we fetched it, which could be a long time ago
        // double mouse_old_y;

        //bool m_mousePressed = false;
        //bool m_leftPressed = false; // True if left key is held down in latest frame
        //bool m_rightPressed = false;
        //bool m_jumpPressed = false;

        // typedef std::function<void(double&, double&)> MousePosFct;
        // typedef std::function<void(Vector3r, double&, double&)> WorldToScreenFct;
        // static MousePosFct mousePosFunc;    // Call to get the coordinates of the mouse onscreen
        // static WorldToScreenFct worldToScreenFunc;  // Call to convert a 3d world point to screen coords


        /*
        // Set the rigidbody's acceleration according to the force being applied on it
        void setExternalForceAcceleration(SimulationModel &model);
        //void setUpcomingPropulsion(Vector3r force);  // Schedules a recoil force next time the physics comes around

        // OpenGL callbacks
        // Called when there's any input. True if the function wants to 'use up' this event
        // Need to be static so they can be set as callbacks
        //static void setMousePosFunc(MousePosFct func) {mousePosFunc = func;}
        //static void setWorldToScreenFunc(WorldToScreenFct func) {worldToScreenFunc = func;}
        // static bool mouseInput(int button, int action, int mods);
        // static bool keyboardInput(int key, int scancode, int action, int mod);

        bool mousePressed(int button, int action, int mods);
        bool keyPressed(unsigned char key);
        bool keyReleased(unsigned char key);
        */

/*
    public:
        unsigned int getControlledObject() { return m_object}; // Gets the index of the object the force is acting on
        Vector3r getCurrentForce() { return m_force; }; // Gets the force being applied on the object
        setCurrentForce(Vector3r force) { m_force = force };    // FIX MEMORY ALLOCATION!!!
};
*/

#endif