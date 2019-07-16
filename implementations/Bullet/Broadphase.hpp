#ifndef APHY_BROADPHASE_HPP_
#define APHY_BROADPHASE_HPP_

#include "Common.hpp"
#include "btBulletDynamicsCommon.h"
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

namespace APhyBullet
{

/**
* Bullet broadphase
*/
struct BulletBroadphase : public aphy::broadphase
{
public:
    BulletBroadphase(btBroadphaseInterface *handle, btGhostPairCallback *ghostPairCallback);
    ~BulletBroadphase();

    btBroadphaseInterface *handle;
    btGhostPairCallback *ghostPairCallback;
};

} // End of namespace APhyBullet

#endif //APHY_BROADPHASE_HPP_
