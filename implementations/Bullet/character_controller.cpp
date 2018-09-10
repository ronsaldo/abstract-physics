#include "character_controller.hpp"
#include "collision_object.hpp"
#include "collision_shape.hpp"
#include "utility.hpp"

_aphy_character_controller::_aphy_character_controller(btKinematicCharacterController *handle)
    : handle(handle), collisionObject(nullptr), collisionShape(nullptr)
{
}

void _aphy_character_controller::lostReferences()
{
    delete handle;

    if(collisionObject)
        collisionObject->release();
    if(collisionShape)
        collisionShape->release();
}

APHY_EXPORT aphy_error aphyAddCharacterControllerReference ( aphy_character_controller* character_controller )
{
    CHECK_POINTER(character_controller);
    return character_controller->retain();
}

APHY_EXPORT aphy_error aphyReleaseCharacterControllerReference ( aphy_character_controller* character_controller )
{
    CHECK_POINTER(character_controller);
    return character_controller->release();
}

APHY_EXPORT aphy_error aphySetCharacterControllerWalkDirection ( aphy_character_controller* character_controller, aphy_vector3 direction )
{
    CHECK_POINTER(character_controller);
    character_controller->handle->setWalkDirection(convertAPhyVector(direction));
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetCharacterControllerWalkDirectionFrom ( aphy_character_controller* character_controller, aphy_vector3* direction )
{
    CHECK_POINTER(direction);
    return aphySetCharacterControllerWalkDirection ( character_controller, *direction );
}

APHY_EXPORT aphy_error aphySetCharacterControllerVelocityForTimeInterval ( aphy_character_controller* character_controller, aphy_vector3 velocity, aphy_scalar time_interval )
{
    CHECK_POINTER(character_controller);
    character_controller->handle->setVelocityForTimeInterval(convertAPhyVector(velocity), time_interval);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetCharacterControllerVelocityForTimeIntervalFrom ( aphy_character_controller* character_controller, aphy_vector3* velocity, aphy_scalar time_interval )
{
    CHECK_POINTER(velocity);
    return aphySetCharacterControllerVelocityForTimeInterval ( character_controller, *velocity, time_interval );
}

APHY_EXPORT aphy_error aphyWarpCharacterController ( aphy_character_controller* character_controller, aphy_vector3 origin )
{
    CHECK_POINTER(character_controller);
    character_controller->handle->warp(convertAPhyVector(origin));
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyWarpCharacterControllerWithOriginFrom ( aphy_character_controller* character_controller, aphy_vector3* origin )
{
    CHECK_POINTER(origin);
    return aphyWarpCharacterController ( character_controller, *origin );
}

APHY_EXPORT aphy_bool aphyCanCharacterControllerJump ( aphy_character_controller* character_controller )
{
    if(!character_controller)
        return false;
    return character_controller->handle->canJump();
}

APHY_EXPORT aphy_error aphyCharacterControllerJump ( aphy_character_controller* character_controller )
{
    CHECK_POINTER(character_controller);
    character_controller->handle->jump();
    return APHY_OK;
}

APHY_EXPORT aphy_bool aphyIsCharacterControllerOnGround ( aphy_character_controller* character_controller )
{
    if(!character_controller)
        return false;
    return character_controller->handle->onGround();
}

APHY_EXPORT aphy_error aphySetCharacterMaxJumpHeight ( aphy_character_controller* character_controller, aphy_scalar height )
{
    CHECK_POINTER(character_controller);
    character_controller->handle->setMaxJumpHeight(height);
    return APHY_OK;
}


APHY_EXPORT aphy_error aphySetCharacterJumpSpeed ( aphy_character_controller* character_controller, aphy_scalar speed )
{
    CHECK_POINTER(character_controller);
    character_controller->handle->setJumpSpeed(speed);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetCharacterGravity ( aphy_character_controller* character_controller, aphy_scalar gravity )
{
    CHECK_POINTER(character_controller);
    character_controller->handle->setGravity(gravity);
    return APHY_OK;
}
