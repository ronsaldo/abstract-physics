#ifndef APHY_BROADPHASE_HPP_
#define APHY_BROADPHASE_HPP_

#include "object.hpp"
#include "btBulletDynamicsCommon.h"

/**
* Bullet broadphase
*/
struct _aphy_broadphase : public Object<_aphy_broadphase>
{
public:
    _aphy_broadphase(btBroadphaseInterface *handle);

    void lostReferences();

    btBroadphaseInterface *handle;
};

#endif //APHY_BROADPHASE_HPP_
