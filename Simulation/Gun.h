#ifndef __GUN__
#define __GUN__

#include "Common/Common.h"
#include "SimulationModel.h"

using namespace PBD;

class Gun {

        static int curBullet;
        static int firstBullet; // Index of the first bullet. Not id, but order of creation in json
        static int numBullets;
    public:
        static void shootBullet(SimulationModel &model, Vector3r bulletPos, Vector3r force);
};

#endif
