#ifndef APHY_COLLISION_OBJECT_HPP_
#define APHY_COLLISION_OBJECT_HPP_

#include "Common.hpp"
#include "btBulletDynamicsCommon.h"

namespace APhyBullet
{

enum class APhyCollisionObjectType
{
    Collision = 0,
    GhostObject,
    PairCachingGhostObject,
    RigidBody,
};

/**
* Bullet broadphase
*/
struct BulletCollisionObject : public aphy::collision_object
{
public:
    BulletCollisionObject(btCollisionObject *handle, APhyCollisionObjectType type);
    ~BulletCollisionObject();

    virtual aphy_transform getTransform() override;
	virtual aphy_error getTransformInto(aphy_transform* result) override;
	virtual aphy_vector3 getTranslation() override;
	virtual aphy_error getTranslationInto(aphy_vector3* result) override;
	virtual aphy_matrix3x3 getMatrix() override;
	virtual aphy_error getMatrixInto(aphy_matrix3x3* result) override;
	virtual aphy_quaternion getQuaternion() override;
	virtual aphy_error getQuaternionInto(aphy_quaternion* result) override;
	virtual aphy_error setTransform(aphy_transform value) override;
	virtual aphy_error setTransformFrom(aphy_transform* value) override;
	virtual aphy_error setTranslation(aphy_vector3 value) override;
	virtual aphy_error setTranslationFrom(aphy_vector3* value) override;
	virtual aphy_error setMatrix(aphy_matrix3x3 value) override;
	virtual aphy_error setMatrixFrom(aphy_matrix3x3* value) override;
	virtual aphy_error setQuaternion(aphy_quaternion value) override;
	virtual aphy_error setQuaternion(aphy_quaternion* value) override;
	virtual aphy_error setCollisionShape(const collision_shape_ref &shape) override;

    btCollisionObject *handle;

    motion_state_ref motionState;
    collision_shape_ref collisionShape;

    APhyCollisionObjectType type;
};

} // End of namespace APhyBullet

#endif //APHY_COLLISION_OBJECT_HPP_
