#include "CollisionObject.hpp"
#include "MotionState.hpp"
#include "CollisionShape.hpp"
#include "Utility.hpp"
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

namespace APhyBullet
{

BulletCollisionObject::BulletCollisionObject(btCollisionObject *handle, APhyCollisionObjectType type)
    : handle(handle), motionState(nullptr), collisionShape(nullptr), type(type)
{
    handle->setUserPointer(this);
}

BulletCollisionObject::~BulletCollisionObject()
{
    delete handle;
}

aphy_transform BulletCollisionObject::getTransform()
{
    return convertTransform(handle->getWorldTransform());
}

aphy_error BulletCollisionObject::getTransformInto(aphy_transform* result)
{
    CHECK_POINTER(result);
    *result = getTransform();
    return APHY_OK;
}

aphy_vector3 BulletCollisionObject::getTranslation()
{
    return convertVector(handle->getWorldTransform().getOrigin());
}

aphy_error BulletCollisionObject::getTranslationInto(aphy_vector3* result)
{
    CHECK_POINTER(result);
    *result = getTranslation();
    return APHY_OK;
}

aphy_matrix3x3 BulletCollisionObject::getMatrix()
{
    return convertMatrix(handle->getWorldTransform().getBasis());
}

aphy_error BulletCollisionObject::getMatrixInto(aphy_matrix3x3* result)
{
    CHECK_POINTER(result);
    *result = getMatrix();
    return APHY_OK;
}

aphy_quaternion BulletCollisionObject::getQuaternion()
{
    return convertQuaternion(handle->getWorldTransform().getRotation());
}

aphy_error BulletCollisionObject::getQuaternionInto(aphy_quaternion* result)
{
    CHECK_POINTER(result);
    *result = getQuaternion();
    return APHY_OK;
}

aphy_error BulletCollisionObject::setTransform(aphy_transform value)
{
    handle->setWorldTransform(convertAPhyTransform(value));
    return APHY_OK;
}

aphy_error BulletCollisionObject::setTransformFrom(aphy_transform* value)
{
    CHECK_POINTER(value);
    return setTransform(*value);
}

aphy_error BulletCollisionObject::setTranslation(aphy_vector3 value)
{
    auto transform = handle->getWorldTransform();
    transform.setOrigin(convertAPhyVector(value));
    handle->setWorldTransform(transform);
    return APHY_OK;
}

aphy_error BulletCollisionObject::setTranslationFrom(aphy_vector3* value)
{
    CHECK_POINTER(value);
    return setTranslation(*value);
}

aphy_error BulletCollisionObject::setMatrix(aphy_matrix3x3 value)
{
    handle->getWorldTransform().setBasis(convertAPhyMatrix(value));
    return APHY_OK;
}

aphy_error BulletCollisionObject::setMatrixFrom(aphy_matrix3x3* value)
{
    CHECK_POINTER(value);
    return setMatrix(*value);
}

aphy_error BulletCollisionObject::setQuaternion(aphy_quaternion value)
{
    handle->getWorldTransform().setRotation(convertAPhyQuaternion(value));
    return APHY_OK;
}

aphy_error BulletCollisionObject::setQuaternion(aphy_quaternion* value)
{
    CHECK_POINTER(value);
    return setQuaternion(*value);
}

aphy_error BulletCollisionObject::setCollisionShape(const collision_shape_ref &shape)
{
    CHECK_POINTER(shape);

    this->collisionShape = shape;
    handle->setCollisionShape(shape.as<BulletCollisionShape>()->handle);
    return APHY_OK;
}

aphy_error BulletCollisionObject::setHasContactResponse(aphy_bool value)
{
    setHandleCollisionFlagValue(btCollisionObject::CF_NO_CONTACT_RESPONSE, !value);
    return APHY_OK;
}

aphy_error BulletCollisionObject::setIsStaticObject(aphy_bool value)
{
    setHandleCollisionFlagValue(btCollisionObject::CF_STATIC_OBJECT, value);
    return APHY_OK;
}

aphy_error BulletCollisionObject::setIsKinematicObject(aphy_bool value)
{
    setHandleCollisionFlagValue(btCollisionObject::CF_KINEMATIC_OBJECT, value);
    return APHY_OK;
}

aphy_error BulletCollisionObject::setIsCharacterObject(aphy_bool value)
{
    setHandleCollisionFlagValue(btCollisionObject::CF_CHARACTER_OBJECT, value);
    return APHY_OK;
}

aphy_error BulletCollisionObject::setDebugDrawingEnabled(aphy_bool value)
{
    setHandleCollisionFlagValue(btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT, !value);
    return APHY_OK;
}

void BulletCollisionObject::setHandleCollisionFlagValue(btCollisionObject::CollisionFlags flag, bool value)
{
    auto flags = handle->getCollisionFlags();
    if(value)
        flags |= flag;
    else
        flags &= ~flag;
    handle->setCollisionFlags(flags);
}

aphy_size BulletCollisionObject::getOverlappingObjectCount()
{
    if(!isGhostObjectType(type))
        return 0;

    return static_cast<btGhostObject*> (handle)->getNumOverlappingObjects();
}

collision_object_ptr BulletCollisionObject::getOverlappingObject(aphy_size index)
{
    if(!isGhostObjectType(type))
        return nullptr;

    auto object = static_cast<btGhostObject*> (handle)->getOverlappingObject(index);
    if(!object)
        return nullptr;

    auto objectWrapper = reinterpret_cast<BulletCollisionObject*> (object->getUserPointer());
    if(!objectWrapper)
        return nullptr;

    return objectWrapper->refFromThis<aphy::collision_object> ().disownedNewRef();
}

} // End of namespace APhyBullet
