#ifndef APHY_WORLD_HPP_
#define APHY_WORLD_HPP_

#include <unordered_set>
#include "object.hpp"
#include "btBulletDynamicsCommon.h"

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

    btDynamicsWorld *handle;
    aphy_collision_dispatcher* collisionDispatcher;
    aphy_broadphase* broadphase;
    aphy_constraint_solver* constraintSolver;
    aphy_collision_configuration* collisionConfiguration;

    std::unordered_set<aphy_collision_object*> collisionObjects;
};

#endif //APHY_WORLD_HPP_
