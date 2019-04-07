#include "Broadphase.hpp"

namespace APhyBullet
{

BulletBroadphase::BulletBroadphase(btBroadphaseInterface *handle)
    : handle(handle)
{
}

BulletBroadphase::~BulletBroadphase()
{
    delete handle;
}

} // End of namespace APhyBullet
