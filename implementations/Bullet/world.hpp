#ifndef APHY_WORLD_HPP_
#define APHY_WORLD_HPP_

#include <unordered_set>
#include "object.hpp"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include <memory>

class aphy_bullet_world_debug_drawer;

/**
* Bullet world
*/
struct _aphy_world : public Object<_aphy_world>
{
public:
    _aphy_world(btDynamicsWorld *handle);

    void lostReferences();
    void addedCollisionObject(aphy_collision_object *object);
    void removedCollisionObject(aphy_collision_object *object);

    aphy_size encodeDebugDrawing();
    aphy_error getDebugDrawingData(aphy_size buffer_size, aphy_pointer buffer );

    btDynamicsWorld *handle;
    aphy_collision_dispatcher* collisionDispatcher;
    aphy_broadphase* broadphase;
    aphy_constraint_solver* constraintSolver;
    aphy_collision_configuration* collisionConfiguration;

    std::unordered_set<aphy_collision_object*> collisionObjects;
    std::shared_ptr<aphy_bullet_world_debug_drawer> debugDrawer;
};

#endif //APHY_WORLD_HPP_
