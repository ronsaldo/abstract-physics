#include "collision_configuration.hpp"

_aphy_collision_configuration::_aphy_collision_configuration(btCollisionConfiguration *handle)
    : handle(handle)
{
}

void _aphy_collision_configuration::lostReferences()
{
    delete handle;
}

// The exported C interface
APHY_EXPORT aphy_error aphyAddCollisionConfigurationReference ( aphy_collision_configuration* collision_configuration )
{
    CHECK_POINTER(collision_configuration);
    return collision_configuration->retain();
}

APHY_EXPORT aphy_error aphyReleaseCollisionConfiguration ( aphy_collision_configuration* collision_configuration )
{
    CHECK_POINTER(collision_configuration);
    return collision_configuration->release();
}
