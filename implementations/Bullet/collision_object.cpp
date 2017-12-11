#include "collision_object.hpp"
#include "motion_state.hpp"
#include "collision_shape.hpp"
#include "utility.hpp"

_aphy_collision_object::_aphy_collision_object(btCollisionObject *handle, bool isRigidBody)
    : handle(handle), motionState(nullptr), collisionShape(nullptr), isRigidBody(isRigidBody)
{
}

void _aphy_collision_object::lostReferences()
{
    delete handle;

    if(motionState)
        motionState->release();
    if(collisionShape)
        collisionShape->release();
}

// The exported C interface
APHY_EXPORT aphy_error aphyAddCollisionObjectReference ( aphy_collision_object* collision_object )
{
    CHECK_POINTER(collision_object);
    return collision_object->retain();
}

APHY_EXPORT aphy_error aphyReleaseCollisionObjectReference ( aphy_collision_object* collision_object )
{
    CHECK_POINTER(collision_object);
    return collision_object->release();
}

APHY_EXPORT aphy_transform aphyGetCollisionObjectTransform ( aphy_collision_object* collision_object )
{
    if(!collision_object)
        return aphy_transform();

    return convertTransform(collision_object->handle->getWorldTransform());
}

APHY_EXPORT aphy_error aphyGetCollisionObjectTransformInto ( aphy_collision_object* collision_object , aphy_transform *result )
{
    CHECK_POINTER(collision_object);
    CHECK_POINTER(result);
    *result = aphyGetCollisionObjectTransform(collision_object);
    return APHY_OK;
}

APHY_EXPORT aphy_vector3 aphyGetCollisionObjectTranslation ( aphy_collision_object* collision_object )
{
    if(collision_object)
        return aphy_vector3();

    return convertVector(collision_object->handle->getWorldTransform().getOrigin());
}

APHY_EXPORT aphy_error aphyGetCollisionObjectTranslationInto ( aphy_collision_object* collision_object , aphy_vector3 *result )
{
    CHECK_POINTER(collision_object);
    CHECK_POINTER(result);
    *result = aphyGetCollisionObjectTranslation(collision_object);
    return APHY_OK;
}

APHY_EXPORT aphy_matrix3x3 aphyGetCollisionObjectMatrix ( aphy_collision_object* collision_object )
{
    if(!collision_object)
        return aphy_matrix3x3();

    return convertMatrix(collision_object->handle->getWorldTransform().getBasis());
}

APHY_EXPORT aphy_error aphyGetCollisionObjectMatrixInto ( aphy_collision_object* collision_object , aphy_matrix3x3 *result )
{
    CHECK_POINTER(collision_object);
    CHECK_POINTER(result);
    *result = aphyGetCollisionObjectMatrix(collision_object);
    return APHY_OK;
}

APHY_EXPORT aphy_quaternion aphyGetCollisionObjectQuaternion ( aphy_collision_object* collision_object )
{
    if(!collision_object)
        return aphy_quaternion();

    return convertQuaternion(collision_object->handle->getWorldTransform().getRotation());
}

APHY_EXPORT aphy_error aphyGetCollisionObjectQuaternionInto ( aphy_collision_object* collision_object , aphy_quaternion *result )
{
    CHECK_POINTER(collision_object);
    CHECK_POINTER(result);
    *result = aphyGetCollisionObjectQuaternion(collision_object);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetCollisionObjectTransform ( aphy_collision_object* collision_object, aphy_transform value )
{
    CHECK_POINTER(collision_object);
    collision_object->handle->setWorldTransform(convertAPhyTransform(value));
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetCollisionObjectTransformFrom ( aphy_collision_object* collision_object, aphy_transform *value )
{
    CHECK_POINTER(collision_object);
    CHECK_POINTER(value);
    return aphySetCollisionObjectTransform(collision_object, *value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectTranslation ( aphy_collision_object* collision_object, aphy_vector3 value )
{
    CHECK_POINTER(collision_object);
    collision_object->handle->getWorldTransform().setOrigin(convertAPhyVector(value));
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetCollisionObjectTranslationFrom ( aphy_collision_object* collision_object, aphy_vector3 *value )
{
    CHECK_POINTER(collision_object);
    CHECK_POINTER(value);
    return aphySetCollisionObjectTranslation(collision_object, *value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectMatrix ( aphy_collision_object* collision_object, aphy_matrix3x3 value )
{
    CHECK_POINTER(collision_object);
    collision_object->handle->getWorldTransform().setBasis(convertAPhyMatrix(value));
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetCollisionObjectMatrixFrom ( aphy_collision_object* collision_object, aphy_matrix3x3 *value )
{
    CHECK_POINTER(collision_object);
    CHECK_POINTER(value);
    return aphySetCollisionObjectMatrix(collision_object, *value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectQuaternion ( aphy_collision_object* collision_object, aphy_quaternion value )
{
    CHECK_POINTER(collision_object);
    collision_object->handle->getWorldTransform().setRotation(convertAPhyQuaternion(value));
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetCollisionObjectQuaternionFrom ( aphy_collision_object* collision_object, aphy_quaternion *value )
{
    CHECK_POINTER(collision_object);
    CHECK_POINTER(value);
    return aphySetCollisionObjectQuaternion(collision_object, *value);
}
