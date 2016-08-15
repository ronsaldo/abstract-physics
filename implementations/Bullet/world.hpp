#ifndef APHY_WORLD_HPP_
#define APHY_WORLD_HPP_

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

    btDynamicsWorld *handle;
    aphy_collision_dispatcher* collision_dispatcher;
    aphy_broadphase* broadphase;
    aphy_constraint_solver* constraint_solver;
    aphy_collision_configuration* collision_configuration;
};

#endif //APHY_WORLD_HPP_
