#include "motion_state.hpp"
#include "utility.hpp"

_aphy_motion_state::_aphy_motion_state(btMotionState *handle)
    : handle(handle)
{
}

void _aphy_motion_state::lostReferences()
{
    delete handle;
}

// The exported C interface
APHY_EXPORT aphy_error aphyAddMotionStateReference ( aphy_motion_state* motion_state )
{
    CHECK_POINTER(motion_state);
    return motion_state->retain();
}

APHY_EXPORT aphy_error aphyReleaseMotionStateReference ( aphy_motion_state* motion_state )
{
    CHECK_POINTER(motion_state);
    return motion_state->release();
}

APHY_EXPORT aphy_transform aphyGetMotionStateTransform ( aphy_motion_state* motion_state )
{
    if(!motion_state)
        return aphy_transform ();

    btTransform transform;
    motion_state->handle->getWorldTransform(transform);
    return convertTransform(transform);
}

APHY_EXPORT aphy_error aphyGetMotionStateTransformInto ( aphy_motion_state* motion_state , aphy_transform *result )
{
    CHECK_POINTER(motion_state);
    CHECK_POINTER(result);
    *result = aphyGetMotionStateTransform(motion_state);
    return APHY_OK;
}

APHY_EXPORT aphy_vector3 aphyGetMotionStateTranslation ( aphy_motion_state* motion_state )
{
    if(!motion_state)
        return aphy_vector3 ();

    btTransform transform;
    motion_state->handle->getWorldTransform(transform);
    return convertVector(transform.getOrigin());
}

APHY_EXPORT aphy_error aphyGetMotionStateTranslationInto ( aphy_motion_state* motion_state , aphy_vector3 *result )
{
    CHECK_POINTER(motion_state);
    CHECK_POINTER(result);
    *result = aphyGetMotionStateTranslation(motion_state);
    return APHY_OK;
}

APHY_EXPORT aphy_matrix3x3 aphyGetMotionStateMatrix ( aphy_motion_state* motion_state )
{
    if(!motion_state)
        return aphy_matrix3x3 ();

    btTransform transform;
    motion_state->handle->getWorldTransform(transform);
    return convertMatrix(transform.getBasis());
}

APHY_EXPORT aphy_error aphyGetMotionStateMatrixInto ( aphy_motion_state* motion_state , aphy_matrix3x3 *result )
{
    CHECK_POINTER(motion_state);
    CHECK_POINTER(result);
    *result = aphyGetMotionStateMatrix(motion_state);
    return APHY_OK;
}

APHY_EXPORT aphy_quaternion aphyGetMotionStateQuaternion ( aphy_motion_state* motion_state )
{
    if(!motion_state)
        return aphy_quaternion ();

    btTransform transform;
    motion_state->handle->getWorldTransform(transform);
    return convertQuaternion(transform.getRotation());
}

APHY_EXPORT aphy_error aphyGetMotionStateQuaternionInto ( aphy_motion_state* motion_state , aphy_quaternion *result )
{
    CHECK_POINTER(motion_state);
    CHECK_POINTER(result);
    *result = aphyGetMotionStateQuaternion(motion_state);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetMotionStateTransform ( aphy_motion_state* motion_state, aphy_transform value )
{
    CHECK_POINTER(motion_state);
    motion_state->handle->setWorldTransform(convertAPhyTransform(value));
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetMotionStateTransformFrom ( aphy_motion_state* motion_state, aphy_transform *value )
{
    CHECK_POINTER(motion_state);
    CHECK_POINTER(value);
    return aphySetMotionStateTransform(motion_state, *value);
}

APHY_EXPORT aphy_error aphySetMotionStateTranslation ( aphy_motion_state* motion_state, aphy_vector3 value )
{
    CHECK_POINTER(motion_state);
    btTransform oldTransform;
    motion_state->handle->getWorldTransform(oldTransform);
    oldTransform.setOrigin(convertAPhyVector(value));
    motion_state->handle->setWorldTransform(oldTransform);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetMotionStateTranslationFrom ( aphy_motion_state* motion_state, aphy_vector3 *value )
{
    CHECK_POINTER(motion_state);
    CHECK_POINTER(value);
    return aphySetMotionStateTranslation(motion_state, *value);
}

APHY_EXPORT aphy_error aphySetMotionStateMatrix ( aphy_motion_state* motion_state, aphy_matrix3x3 value )
{
    CHECK_POINTER(motion_state);
    btTransform oldTransform;
    motion_state->handle->getWorldTransform(oldTransform);
    oldTransform.setBasis(convertAPhyMatrix(value));
    motion_state->handle->setWorldTransform(oldTransform);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetMotionStateMatrixFrom ( aphy_motion_state* motion_state, aphy_matrix3x3 *value )
{
    CHECK_POINTER(motion_state);
    CHECK_POINTER(value);
    return aphySetMotionStateMatrix(motion_state, *value);
}

APHY_EXPORT aphy_error aphySetMotionStateQuaternion ( aphy_motion_state* motion_state, aphy_quaternion value )
{
    CHECK_POINTER(motion_state);
    btTransform oldTransform;
    motion_state->handle->getWorldTransform(oldTransform);
    oldTransform.setRotation(convertAPhyQuaternion(value));
    motion_state->handle->setWorldTransform(oldTransform);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetMotionStateQuaternionFrom ( aphy_motion_state* motion_state, aphy_quaternion *value )
{
    CHECK_POINTER(motion_state);
    CHECK_POINTER(value);
    return aphySetMotionStateQuaternion(motion_state, *value);
}
