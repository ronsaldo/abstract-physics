#include "MotionState.hpp"
#include "Utility.hpp"

namespace APhyBullet
{

BulletMotionState::BulletMotionState(btMotionState *handle)
    : handle(handle)
{
}

BulletMotionState::~BulletMotionState()
{
    delete handle;
}

aphy_transform BulletMotionState::getTransform()
{
    btTransform transform;
    handle->getWorldTransform(transform);
    return convertTransform(transform);
}

aphy_error BulletMotionState::getTransformInto(aphy_transform* result)
{
    CHECK_POINTER(result);
    *result = getTransform();
    return APHY_OK;
}

aphy_vector3 BulletMotionState::getTranslation()
{
    btTransform transform;
    handle->getWorldTransform(transform);
    return convertVector(transform.getOrigin());
}

aphy_error BulletMotionState::getTranslationInto(aphy_vector3* result)
{
    CHECK_POINTER(result);
    *result = getTranslation();
    return APHY_OK;
}

aphy_matrix3x3 BulletMotionState::getMatrix()
{
    btTransform transform;
    handle->getWorldTransform(transform);
    return convertMatrix(transform.getBasis());
}

aphy_error BulletMotionState::getMatrixInto(aphy_matrix3x3* result)
{
    CHECK_POINTER(result);
    *result = getMatrix();
    return APHY_OK;
}

aphy_quaternion BulletMotionState::getQuaternion()
{
    btTransform transform;
    handle->getWorldTransform(transform);
    return convertQuaternion(transform.getRotation());
}

aphy_error BulletMotionState::getQuaternionInto(aphy_quaternion* result)
{
    CHECK_POINTER(result);
    *result = getQuaternion();
    return APHY_OK;
}

aphy_error BulletMotionState::setTransform(aphy_transform value)
{
    // TODO: Implement this.
    return APHY_OK;
}

aphy_error BulletMotionState::setTransformFrom(aphy_transform* value)
{
    CHECK_POINTER(value);
    return setTransform(*value);
}

aphy_error BulletMotionState::setTranslation(aphy_vector3 value)
{
    btTransform oldTransform;
    handle->getWorldTransform(oldTransform);
    oldTransform.setOrigin(convertAPhyVector(value));
    handle->setWorldTransform(oldTransform);
    return APHY_OK;
}

aphy_error BulletMotionState::setTranslationFrom(aphy_vector3* value)
{
    CHECK_POINTER(value);
    return setTranslation(*value);
}

aphy_error BulletMotionState::setMatrix(aphy_matrix3x3 value)
{
    btTransform oldTransform;
    handle->getWorldTransform(oldTransform);
    oldTransform.setBasis(convertAPhyMatrix(value));
    handle->setWorldTransform(oldTransform);
    return APHY_OK;
}

aphy_error BulletMotionState::setMatrixFrom(aphy_matrix3x3* value)
{
    CHECK_POINTER(value);
    return setMatrix(*value);
}

aphy_error BulletMotionState::setQuaternion(aphy_quaternion value)
{
    btTransform oldTransform;
    handle->getWorldTransform(oldTransform);
    oldTransform.setRotation(convertAPhyQuaternion(value));
    handle->setWorldTransform(oldTransform);
    return APHY_OK;
}

aphy_error BulletMotionState::setQuaternionFrom(aphy_quaternion* value)
{
    CHECK_POINTER(value);
    return setQuaternion(*value);
}

} // End of namespace APhyBullet
