#ifndef APHY_COLLISION_DISPATCHER_HPP_
#define APHY_COLLISION_DISPATCHER_HPP_

#include "object.hpp"
#include "btBulletDynamicsCommon.h"

/**
* Bullet collision dispatcher
*/
struct _aphy_collision_dispatcher : public Object<_aphy_collision_dispatcher>
{
public:
    _aphy_collision_dispatcher(btCollisionDispatcher *handle, aphy_collision_configuration *configuration);

    void lostReferences();

    btCollisionDispatcher *handle;
    aphy_collision_configuration *configuration;
};

#endif //APHY_COLLISION_DISPATCHER_HPP_
