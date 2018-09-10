#ifndef APHY_COLLISION_OBJECT_HPP_
#define APHY_COLLISION_OBJECT_HPP_

#include "object.hpp"
#include "btBulletDynamicsCommon.h"

enum class APhyCollisionObjectType
{
    Collision = 0,
    GhostObject,
    PairCachingGhostObject,
    RigidBody,
};

/**
* Bullet broadphase
*/
struct _aphy_collision_object : public Object<_aphy_collision_object>
{
public:
    _aphy_collision_object(btCollisionObject *handle, APhyCollisionObjectType type);

    void lostReferences();

    btCollisionObject *handle;

    aphy_motion_state *motionState;
    aphy_collision_shape *collisionShape;

    APhyCollisionObjectType type;
};

#endif //APHY_COLLISION_OBJECT_HPP_
