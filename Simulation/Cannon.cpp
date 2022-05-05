#include "Cannon.h"

#include "RigidBody.h"

int Cannon::firstBullet = 1;
int Cannon::curBullet = Cannon::firstBullet;
int Cannon::numBullets = 5;

void Cannon::shootBullet(SimulationModel &model, Vector3r bulletPos, Vector3r force) {
    if (curBullet >= firstBullet + numBullets)
        curBullet = firstBullet;

    // Get next bullet
    RigidBody *bullet = model.getRigidBodies()[curBullet];
    curBullet++;

    Vector3r &pos = bullet->getPosition();

    pos.x() = bulletPos.x();
    pos.y() = bulletPos.y();
    pos.z() = bulletPos.z();

    // Clear velocity and acceleration in case bullet happened to be moving
    Vector3r &acc = bullet->getAcceleration();
    acc.x() = 0;
    acc.y() = 0;
    acc.z() = 0;

    Vector3r &vel = bullet->getVelocity();
    vel.x() = 0;
    vel.y() = 0;
    vel.z() = 0;

    // Set mass, because bullets are made static at the start
    bullet->setMass(100.0);
    
    acc += force*4;
}
