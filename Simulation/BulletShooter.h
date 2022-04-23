#ifndef __BULLET_SHOOTER_H__
#define __BULLET_SHOOTER_H__

#include "Common/Common.h"
#include "SimulationModel.h"
#include "RigidBody.h"

class BullerShooter {
    public:
        // Singleton
        static ForceController* getCurrent();
        static void setCurrent(ForceController* controller);

        void setControlledObject (int index);

        static bool mouseInput();
}

#endif