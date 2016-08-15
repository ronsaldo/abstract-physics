#ifndef APHY_COLLISION_CONFIGURATION_HPP_
#define APHY_COLLISION_CONFIGURATION_HPP_

#include "object.hpp"
#include "btBulletDynamicsCommon.h"

/**
* Bullet collision configuration
*/
struct _aphy_collision_configuration : public Object<_aphy_collision_configuration>
{
public:
    _aphy_collision_configuration(btCollisionConfiguration *handle);

    void lostReferences();

    btCollisionConfiguration *handle;
};

#endif //APHY_BROADPHASE_HPP_
