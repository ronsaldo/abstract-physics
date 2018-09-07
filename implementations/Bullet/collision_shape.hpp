#ifndef APHY_COLLISION_SHAPE_HPP_
#define APHY_COLLISION_SHAPE_HPP_

#include "object.hpp"
#include "btBulletDynamicsCommon.h"

enum class APhyBulletCollisionShapeType
{
    Generic = 0,
    Compound,
    HeightField,
};

/**
* Bullet collision shaoe
*/
struct _aphy_collision_shape : public Object<_aphy_collision_shape>
{
public:
    _aphy_collision_shape(btCollisionShape *handle, APhyBulletCollisionShapeType shapeType = APhyBulletCollisionShapeType::Generic);

    void lostReferences();

    btCollisionShape *handle;
    APhyBulletCollisionShapeType shapeType;
};

#endif //APHY_COLLISION_SHAPE_HPP_
