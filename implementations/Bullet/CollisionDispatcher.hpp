#ifndef APHY_COLLISION_DISPATCHER_HPP_
#define APHY_COLLISION_DISPATCHER_HPP_

#include "Common.hpp"
#include "btBulletDynamicsCommon.h"

namespace APhyBullet
{

/**
* Bullet collision dispatcher
*/
struct BulletCollisionDispatcher : public aphy::collision_dispatcher
{
public:
    BulletCollisionDispatcher(btCollisionDispatcher *handle, const collision_configuration_ref &configuration);
    ~BulletCollisionDispatcher();

    btCollisionDispatcher *handle;
    collision_configuration_ref configuration;
};

} // End of namespace APhyBullet

#endif //APHY_COLLISION_DISPATCHER_HPP_
