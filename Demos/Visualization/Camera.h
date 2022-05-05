#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "Common/Common.h"
#include "Simulation/SimulationModel.h"

// A simple camera.
//  It attempts to always have the player at the centre of the screen
//  It doesn't move until the player gets a certain distance from the center
//      along a certain quadrant. Then it follows. After that, if the player
//      moves in the opposite x direction, the camera stays still, until
//      the player moves a certain distance in that direction.
//  Every time the camera starts moving again, it spends the next few frames
//      catching up to the player's position
class Camera {
    private:
        static Camera* current;
        static int playerObj;

        int m_counter = 0;
        bool isFirstUpdate = true;
        PBD::SimulationModel *m_model;
        int m_screenW;
        int m_screenH;
        double m_oldPlayerScreenPosX;
        double m_oldPlayerScreenPosY;
        int m_xMoveDir;
        int m_yMoveDir;
        bool m_currentlyMoving = true;
        Vector2r m_freezePosition;

        static double moveThreshold;
        static double maxMoveDist;
        static double minMoveDist;

    public:
        // Singleton
        static Camera* getCurrent();

        void init(PBD::SimulationModel *model, int playerObject);
        void update();
};

#endif
