#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "Common/Common.h"

class Camera {
    private:
        static Camera* m_current;
        static int m_controlledObject;
        static std::function<void(Real, Real, Real)> moveFunc;
    public:
        // Singleton
        static Camera* getCurrent();

        void setMoveFunc(std::function<void(Real, Real, Real)> func);

        void setControlledObject(int object);
        void update();
};

#endif