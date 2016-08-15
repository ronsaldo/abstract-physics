#include "collision_dispatcher.hpp"
#include "collision_configuration.hpp"

_aphy_collision_dispatcher::_aphy_collision_dispatcher(btCollisionDispatcher *handle, aphy_collision_configuration *configuration)
    : handle(handle), configuration(configuration)
{
    if(configuration)
        configuration->retain();
}

void _aphy_collision_dispatcher::lostReferences()
{
    delete handle;
    if(configuration)
        configuration->release();
}

// The exported C interface
APHY_EXPORT aphy_error aphyAddCollisionDispatcherReference ( aphy_collision_dispatcher* collision_dispatcher )
{
    CHECK_POINTER(collision_dispatcher);
    return collision_dispatcher->retain();
}

APHY_EXPORT aphy_error aphyReleaseCollisionDispatcher ( aphy_collision_dispatcher* collision_dispatcher )
{
    CHECK_POINTER(collision_dispatcher);
    return collision_dispatcher->release();
}
