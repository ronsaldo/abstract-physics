#ifndef APHY_MOTION_STATE_HPP_
#define APHY_MOTION_STATE_HPP_

#include "object.hpp"
#include "btBulletDynamicsCommon.h"

/**
* Bullet motion state
*/
struct _aphy_motion_state : public Object<_aphy_motion_state>
{
public:
    _aphy_motion_state(btMotionState *handle);

    void lostReferences();

    btMotionState *handle;
};

#endif //APHY_MOTION_STATE_HPP_
