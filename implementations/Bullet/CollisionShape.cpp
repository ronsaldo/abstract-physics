#include "CollisionShape.hpp"
#include "Utility.hpp"

namespace APhyBullet
{

BulletCollisionShape::BulletCollisionShape(btCollisionShape *handle, APhyBulletCollisionShapeType shapeType)
    : handle(handle), shapeType(shapeType)
{
}

BulletCollisionShape::~BulletCollisionShape()
{
    delete handle;
}

aphy_error BulletCollisionShape::setMargin(aphy_scalar margin)
{
    handle->setMargin(margin);
    return APHY_OK;

}

aphy_scalar BulletCollisionShape::getMargin()
{
    return handle->getMargin();
}

aphy_vector3 BulletCollisionShape::computeLocalInertia(aphy_scalar mass)
{
    btVector3 result;
    handle->calculateLocalInertia(mass, result);
    return convertVector(result);
}

aphy_error BulletCollisionShape::computeLocalInertiaInto(aphy_scalar mass, aphy_vector3* result)
{
    CHECK_POINTER(result);
    *result = computeLocalInertia(mass);
    return APHY_OK;
}

aphy_error BulletCollisionShape::addLocalShapeWithTransform(const collision_shape_ref &shape, aphy_transform transform)
{
    CHECK_POINTER(shape);
    if(shapeType != APhyBulletCollisionShapeType::Compound)
        return APHY_UNSUPPORTED;

    auto compound = static_cast<btCompoundShape*> (handle);
    compound->addChildShape(convertAPhyTransform(transform), shape.as<BulletCollisionShape>()->handle);
    return APHY_OK;

}

aphy_error BulletCollisionShape::addLocalShapeWithTransformFrom(const collision_shape_ref &shape, aphy_transform* transform)
{
    CHECK_POINTER(transform);
    return addLocalShapeWithTransform(shape, *transform);
}

} // End of namespace APhyBullet
