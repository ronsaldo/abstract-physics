#include "collision_shape.hpp"
#include "utility.hpp"

_aphy_collision_shape::_aphy_collision_shape(btCollisionShape *handle, APhyBulletCollisionShapeType shapeType)
    : handle(handle), shapeType(shapeType)
{
}

void _aphy_collision_shape::lostReferences()
{
    delete handle;
}

// The exported C interface
APHY_EXPORT aphy_error aphyAddCollisionShapeReference ( aphy_collision_shape* collision_shape )
{
    CHECK_POINTER(collision_shape);
    return collision_shape->retain();
}

APHY_EXPORT aphy_error aphyReleaseCollisionShapeReference ( aphy_collision_shape* collision_shape )
{
    CHECK_POINTER(collision_shape);
    return collision_shape->release();
}

APHY_EXPORT aphy_error aphySetShapeMargin ( aphy_collision_shape* collision_shape, aphy_scalar margin )
{
    CHECK_POINTER(collision_shape);
    collision_shape->handle->setMargin(margin);
    return APHY_OK;
}

APHY_EXPORT aphy_scalar aphyGetShapeMargin ( aphy_collision_shape* collision_shape )
{
    if(!collision_shape)
        return 0;
    return collision_shape->handle->getMargin();
}

APHY_EXPORT aphy_vector3 aphyComputeLocalInertia ( aphy_collision_shape* collision_shape, aphy_scalar mass )
{
    if(!collision_shape)
        return aphy_vector3();

    btVector3 result;
    collision_shape->handle->calculateLocalInertia(mass, result);
    return convertVector(result);
}

APHY_EXPORT aphy_error aphyComputeLocalInertiaInto ( aphy_collision_shape* collision_shape, aphy_scalar mass, aphy_vector3 *result)
{
    CHECK_POINTER(collision_shape);
    CHECK_POINTER(result);
    *result = aphyComputeLocalInertia(collision_shape, mass);
    return APHY_OK;
}


APHY_EXPORT aphy_error aphyAddLocalShapeWithTransform ( aphy_collision_shape* collision_shape, aphy_collision_shape* shape, aphy_transform transform )
{
    CHECK_POINTER(collision_shape);
    CHECK_POINTER(shape);
    if(collision_shape->shapeType != APhyBulletCollisionShapeType::Compound)
        return APHY_UNSUPPORTED;

    auto compound = static_cast<btCompoundShape*> (collision_shape->handle);
    compound->addChildShape(convertAPhyTransform(transform), shape->handle);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyAddLocalShapeWithTransformFrom ( aphy_collision_shape* collision_shape, aphy_collision_shape* shape, aphy_transform* transform )
{
    CHECK_POINTER(transform);
    return aphyAddLocalShapeWithTransform(collision_shape, shape, *transform);
}
