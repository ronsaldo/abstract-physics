#ifndef APHY_MOTION_STATE_HPP_
#define APHY_MOTION_STATE_HPP_

#include "Common.hpp"
#include "btBulletDynamicsCommon.h"

namespace APhyBullet
{

/**
* Bullet motion state
*/
struct BulletMotionState : public aphy::motion_state
{
public:
    BulletMotionState(btMotionState *handle);
    ~BulletMotionState();

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
	virtual aphy_error setQuaternionFrom(aphy_quaternion* value) override;

    btMotionState *handle;
};

} // End of namespace APhyBullet

#endif //APHY_MOTION_STATE_HPP_
