#include "CollisionConfiguration.hpp"

namespace APhyBullet
{
BulletCollisionConfiguration::BulletCollisionConfiguration(btCollisionConfiguration *handle)
    : handle(handle)
{
}

BulletCollisionConfiguration::~BulletCollisionConfiguration()
{
    delete handle;
}

} // End of namespace APhyBullet
