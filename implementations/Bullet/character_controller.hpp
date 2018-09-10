#ifndef APHY_CHARACTER_CONTROLLER_HPP_
#define APHY_CHARACTER_CONTROLLER_HPP_

#include "object.hpp"
#include "btBulletDynamicsCommon.h"
#include <BulletDynamics/Character/btKinematicCharacterController.h>

/**
* Bullet character controller interface
*/
struct _aphy_character_controller : public Object<_aphy_character_controller>
{
public:
    _aphy_character_controller(btKinematicCharacterController *handle);

    void lostReferences();

    btKinematicCharacterController *handle;
    aphy_collision_object *collisionObject;
    aphy_collision_shape *collisionShape;
};

#endif //APHY_COLLISION_OBJECT_HPP_
