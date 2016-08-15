#include "collision_object.hpp"
#include "motion_state.hpp"
#include "collision_shape.hpp"

_aphy_collision_object::_aphy_collision_object(btCollisionObject *handle, bool isRigidBody)
    : handle(handle), motionState(nullptr), collisionShape(nullptr), isRigidBody(isRigidBody)
{
}

void _aphy_collision_object::lostReferences()
{
    delete handle;

    if(motionState)
        motionState->release();
    if(collisionShape)
        collisionShape->release();
}

// The exported C interface
APHY_EXPORT aphy_error aphyAddCollisionObjectReference ( aphy_collision_object* collision_object )
{
    CHECK_POINTER(collision_object);
    return collision_object->retain();
}

APHY_EXPORT aphy_error aphyReleaseCollisionObjectReference ( aphy_collision_object* collision_object )
{
    CHECK_POINTER(collision_object);
    return collision_object->release();
}
