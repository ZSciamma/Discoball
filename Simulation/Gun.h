#ifndef __GUN__
#define __GUN__

#include "Common/Common.h"
#include "SimulationModel.h"

using namespace PBD;

class Gun {
    public:
        static void shootBullet(SimulationModel &model, Vector3r bulletPos, Vector3r force);
};

#endif
