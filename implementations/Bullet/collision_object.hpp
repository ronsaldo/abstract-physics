#ifndef APHY_COLLISION_OBJECT_HPP_
#define APHY_COLLISION_OBJECT_HPP_

#include "object.hpp"
#include "btBulletDynamicsCommon.h"

/**
* Bullet broadphase
*/
struct _aphy_collision_object : public Object<_aphy_collision_object>
{
public:
    _aphy_collision_object(btCollisionObject *handle, bool isRigidBody);

    void lostReferences();

    btCollisionObject *handle;
    bool isRigidBody;
};

#endif //APHY_COLLISION_OBJECT_HPP_
