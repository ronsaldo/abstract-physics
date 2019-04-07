#include "CollisionDispatcher.hpp"
#include "CollisionConfiguration.hpp"

namespace APhyBullet
{

BulletCollisionDispatcher::BulletCollisionDispatcher(btCollisionDispatcher *chandle, const collision_configuration_ref &cconfiguration)
    : handle(chandle), configuration(cconfiguration)
{
}

BulletCollisionDispatcher::~BulletCollisionDispatcher()
{
    delete handle;
}


} // End of namespace APhyBullet
