#ifndef APHY_COLLISION_SHAPE_HPP_
#define APHY_COLLISION_SHAPE_HPP_

#include "Common.hpp"
#include "btBulletDynamicsCommon.h"

namespace APhyBullet
{

enum class APhyBulletCollisionShapeType
{
    Generic = 0,
    Compound,
    HeightField,
};

/**
* Bullet collision shaoe
*/
struct BulletCollisionShape : public aphy::collision_shape
{
public:
    BulletCollisionShape(btCollisionShape *handle, APhyBulletCollisionShapeType shapeType = APhyBulletCollisionShapeType::Generic);
    ~BulletCollisionShape();

    virtual aphy_error setMargin(aphy_scalar margin) override;
	virtual aphy_scalar getMargin() override;
	virtual aphy_vector3 computeLocalInertia(aphy_scalar mass) override;
	virtual aphy_error computeLocalInertiaInto(aphy_scalar mass, aphy_vector3* result) override;
	virtual aphy_error addLocalShapeWithTransform(const collision_shape_ref &shape, aphy_transform transform) override;
	virtual aphy_error addLocalShapeWithTransformFrom(const collision_shape_ref &shape, aphy_transform* transform) override;

    btCollisionShape *handle;
    APhyBulletCollisionShapeType shapeType;
};

} // End of namespace APhyBullet

#endif //APHY_COLLISION_SHAPE_HPP_
