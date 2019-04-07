#include "CharacterController.hpp"
#include "CollisionObject.hpp"
#include "CollisionShape.hpp"
#include "Utility.hpp"

namespace APhyBullet
{

BulletCharacterController::BulletCharacterController(btKinematicCharacterController *handle)
    : handle(handle), collisionObject(nullptr), collisionShape(nullptr)
{
}

BulletCharacterController::~BulletCharacterController()
{
    delete handle;
}

aphy_error BulletCharacterController::setWalkDirection(aphy_vector3 direction)
{
    handle->setWalkDirection(convertAPhyVector(direction));
    return APHY_OK;
}

aphy_error BulletCharacterController::setWalkDirectionFrom(aphy_vector3* direction)
{
    CHECK_POINTER(direction);
    return setWalkDirection(*direction);
}

aphy_error BulletCharacterController::setVelocityForTimeInterval(aphy_vector3 velocity, aphy_scalar time_interval)
{
    handle->setVelocityForTimeInterval(convertAPhyVector(velocity), time_interval);
    return APHY_OK;
}

aphy_error BulletCharacterController::setVelocityForTimeIntervalFrom(aphy_vector3* velocity, aphy_scalar time_interval)
{
    CHECK_POINTER(velocity);
    return setVelocityForTimeInterval(*velocity, time_interval );
}

aphy_error BulletCharacterController::warp(aphy_vector3 origin)
{
    handle->warp(convertAPhyVector(origin));
    return APHY_OK;
}

aphy_error BulletCharacterController::warpWithOriginFrom(aphy_vector3* origin)
{
    CHECK_POINTER(origin);
    return warp(*origin);
}

aphy_bool BulletCharacterController::canJump()
{
    return handle->canJump();
}

aphy_error BulletCharacterController::jump()
{
    handle->jump();
    return APHY_OK;
}

aphy_bool BulletCharacterController::isOnGround()
{
    return handle->onGround();
}

aphy_error BulletCharacterController::setMaxJumpHeight(aphy_scalar height)
{
    handle->setMaxJumpHeight(height);
    return APHY_OK;
}

aphy_error BulletCharacterController::setJumpSpeed(aphy_scalar speed)
{
    handle->setJumpSpeed(speed);
    return APHY_OK;
}

aphy_error BulletCharacterController::setGravity(aphy_scalar gravity)
{
    handle->setGravity(gravity);
    return APHY_OK;
}

} // End of namespace APhyBullet
