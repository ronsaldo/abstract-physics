#include "broadphase.hpp"

_aphy_broadphase::_aphy_broadphase(btBroadphaseInterface *handle)
    : handle(handle)
{
}

void _aphy_broadphase::lostReferences()
{
    delete handle;
}

// The exported C interface
APHY_EXPORT aphy_error aphyAddBroadphaseReference ( aphy_broadphase* broadphase )
{
    CHECK_POINTER(broadphase);
    return broadphase->retain();
}

APHY_EXPORT aphy_error aphyReleaseBroadphaseReference ( aphy_broadphase* broadphase )
{
    CHECK_POINTER(broadphase);
    return broadphase->release();
}
