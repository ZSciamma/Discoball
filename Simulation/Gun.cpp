#include "Gun.h"

#include "RigidBody.h"

int Gun::firstBullet = 1;
int Gun::curBullet = Gun::firstBullet;
int Gun::numBullets = 5;

void Gun::shootBullet(SimulationModel &model, Vector3r bulletPos, Vector3r force) {
    if (curBullet >= firstBullet + numBullets)
        curBullet = firstBullet;

    // Get next bullet
    //SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
    RigidBody *bullet = model.getRigidBodies()[curBullet];
    curBullet++;

    Vector3r &pos = bullet->getPosition();

    pos.x() = bulletPos.x();
    pos.y() = bulletPos.y();
    pos.z() = bulletPos.z();

    //std::cout << "New bullet pos: " << pos.x() << ", " << pos.y() << ", " << pos.z() << std::endl;

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
    
    //std::cout << "Acceleration before: " << acc.x() << ", " << acc.y() << ", " << acc.z() << std::endl;
    acc += force*4;
    //std::cout << "Acceleration after: " << acc.x() << ", " << acc.y() << ", " << acc.z() << std::endl;
}


/*
// Called by GL when the mouse is clicked
bool BulletShooter:mouseInput() {
    // Prepare teleportation
    // Select the bullet we're going to use?
    // In any case mark that, when the TimeStepController next calls us,
    //  we're going to teleport a bullet.
    return false; // ?
}
*/

// Called by the physics when it's ready to teleport the bullet into place

    // Choose the final location of the bullet (do this here or in the mouseInput function?)

    // Check if location is right? Otherwise maybe just don't create a bullet? Unclear

    // Place bullet

    // If the bullet is shooting into a wall or something, maybe we should wait for
    //  the cube to be pushed away and only then instantiate the bullet? Or is there
    //  a certain tuning of parameters which will make the physics do that by itself
    //  if we place the bullet where there would be a collision? I'm afraid it would
    //  just get stuck in a wall or something, which might not be great for the user.
    //  I.e. the cube is pushed away and the bullet kind of appears in its place
    // Since we *know* the cube is in a good location, we could actually wait a couple
    //  frames for the cube to be shot away, which we know it will be unless it's against
    //  a rock and a hard place or something, and only then (a couple of frames later)
    //  put the bullet where the cube used to be, and have it shoot in the opposite direction.
