#include "Broadphase.hpp"

namespace APhyBullet
{

BulletBroadphase::BulletBroadphase(btBroadphaseInterface *handle, btGhostPairCallback *ghostPairCallback)
    : handle(handle), ghostPairCallback(ghostPairCallback)
{
}

BulletBroadphase::~BulletBroadphase()
{
    delete ghostPairCallback;
    delete handle;

}

} // End of namespace APhyBullet
