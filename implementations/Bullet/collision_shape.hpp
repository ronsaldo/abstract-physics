#ifndef APHY_COLLISION_SHAPE_HPP_
#define APHY_COLLISION_SHAPE_HPP_

#include "object.hpp"
#include "btBulletDynamicsCommon.h"

/**
* Bullet collision shaoe
*/
struct _aphy_collision_shape : public Object<_aphy_collision_shape>
{
public:
    _aphy_collision_shape(btCollisionShape *handle);

    void lostReferences();

    btCollisionShape *handle;
};

#endif //APHY_COLLISION_SHAPE_HPP_
