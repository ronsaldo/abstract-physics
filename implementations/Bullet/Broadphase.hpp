#ifndef APHY_BROADPHASE_HPP_
#define APHY_BROADPHASE_HPP_

#include "Common.hpp"
#include "btBulletDynamicsCommon.h"

namespace APhyBullet
{

/**
* Bullet broadphase
*/
struct BulletBroadphase : public aphy::broadphase
{
public:
    BulletBroadphase(btBroadphaseInterface *handle);
    ~BulletBroadphase();

    btBroadphaseInterface *handle;
};

} // End of namespace APhyBullet

#endif //APHY_BROADPHASE_HPP_
