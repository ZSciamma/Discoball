#include "BulletShooter.h"

// Called by GL when the mouse is clicked
bool BulletShooter:mouseInput() {
    // Prepare teleportation
    // Select the bullet we're going to use?
    // In any case mark that, when the TimeStepController next calls us,
    //  we're going to teleport a bullet.
    return false; // ?
}

// Called by the physics when it's ready to teleport the bullet into place
bool BulletShooter:actuateTeleportation() {
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

    // Also can we get the force from the ForceController so we can just throw the bullet in
    //  the opposite direction? Would be good to have a Player class or something which 
    //  contains both BulletShooter and ForceController so we can just have the result
    //  of the calculations passed down
}
