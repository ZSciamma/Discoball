#ifndef __Thrusters_h__
#define __Thrusters_h__

#include "Common/Common.h"
#include "SimulationModel.h"

using namespace PBD;

// Monitors the amount of external force currently applied on an object
//  Force might be applied through some external input
class Thrusters {
    private:
        static Thrusters* current;

        int m_controlledObj = -1;

    public:
        // Singleton
        static Thrusters* getCurrent();
        static void setCurrent(Thrusters* controller);

        void setControlledObject (int index);

        // Recoil
        void applyPropulsion(SimulationModel &model, Vector3r force);  // Physics is ready for force to be applied
};

#endif
