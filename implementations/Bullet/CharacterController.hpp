#ifndef APHY_CHARACTER_CONTROLLER_HPP_
#define APHY_CHARACTER_CONTROLLER_HPP_

#include "Common.hpp"
#include "btBulletDynamicsCommon.h"
#include <BulletDynamics/Character/btKinematicCharacterController.h>

namespace APhyBullet
{

/**
* Bullet character controller interface
*/
struct BulletCharacterController : public aphy::character_controller
{
public:
    BulletCharacterController(btKinematicCharacterController *handle);
    ~BulletCharacterController();

    virtual aphy_error setWalkDirection(aphy_vector3 direction) override;
	virtual aphy_error setWalkDirectionFrom(aphy_vector3* direction) override;
	virtual aphy_error setVelocityForTimeInterval(aphy_vector3 velocity, aphy_scalar time_interval) override;
	virtual aphy_error setVelocityForTimeIntervalFrom(aphy_vector3* velocity, aphy_scalar time_interval) override;
	virtual aphy_error warp(aphy_vector3 origin) override;
	virtual aphy_error warpWithOriginFrom(aphy_vector3* origin) override;
	virtual aphy_bool canJump() override;
	virtual aphy_error jump() override;
	virtual aphy_bool isOnGround() override;
	virtual aphy_error setMaxJumpHeight(aphy_scalar height) override;
	virtual aphy_error setJumpSpeed(aphy_scalar speed) override;
	virtual aphy_error setGravity(aphy_scalar gravity) override;

    btKinematicCharacterController *handle;
    collision_object_ref collisionObject;
    collision_shape_ref collisionShape;
};

} // End of namespace APhyBullet

#endif //APHY_COLLISION_OBJECT_HPP_
