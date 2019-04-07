#ifndef APHY_COLLISION_CONFIGURATION_HPP_
#define APHY_COLLISION_CONFIGURATION_HPP_

#include "Common.hpp"
#include "btBulletDynamicsCommon.h"

namespace APhyBullet
{

/**
* Bullet collision configuration
*/
struct BulletCollisionConfiguration : public aphy::collision_configuration
{
public:
    BulletCollisionConfiguration(btCollisionConfiguration *handle);
    ~BulletCollisionConfiguration();

    btCollisionConfiguration *handle;
};

} // End of namespace APhyBullet

#endif //APHY_BROADPHASE_HPP_
