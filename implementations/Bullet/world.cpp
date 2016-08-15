#include "collision_object.hpp"
#include "world.hpp"

_aphy_world::_aphy_world(btDynamicsWorld *handle)
    : handle(handle)
{
}

void _aphy_world::lostReferences()
{
    delete handle;
}

// The exported C interface
APHY_EXPORT aphy_error aphyAddWorldReference ( aphy_world* world )
{
    CHECK_POINTER(world);
    return world->retain();
}

APHY_EXPORT aphy_error aphyReleaseWorldReference ( aphy_world* world )
{
    CHECK_POINTER(world);
    return world->release();
}

APHY_EXPORT aphy_uint aphyGetNumberOfCollisionObject ( aphy_world* world )
{
    if(!world)
        return 0;

    return world->handle->getNumCollisionObjects();
}

APHY_EXPORT aphy_uint aphyGetNumberOfConstraints ( aphy_world* world )
{
    if(!world)
        return 0;

    return world->handle->getNumConstraints();
}

APHY_EXPORT aphy_error aphyAddCollisionObject ( aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask )
{
    CHECK_POINTER(world);
    world->handle->addCollisionObject(object->handle, collision_filter_group, collision_filter_mask);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyRemoveCollisionObject ( aphy_world* world, aphy_collision_object* object )
{
    CHECK_POINTER(world);
    CHECK_POINTER(object);
    world->handle->removeCollisionObject(object->handle);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyAddRigidBody ( aphy_world* world, aphy_collision_object* object )
{
    CHECK_POINTER(world);
    CHECK_POINTER(object);

    auto rigidBody = btRigidBody::upcast(object->handle);
    if(!rigidBody)
        return APHY_INVALID_PARAMETER;

    world->handle->addRigidBody(rigidBody);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyRemoveRigidBody ( aphy_world* world, aphy_collision_object* object )
{
    CHECK_POINTER(world);
    CHECK_POINTER(object);

    auto rigidBody = btRigidBody::upcast(object->handle);
    if(!rigidBody)
        return APHY_INVALID_PARAMETER;

    world->handle->removeRigidBody(rigidBody);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyAddRigidBodyWithFilter ( aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask )
{
    CHECK_POINTER(world);
    CHECK_POINTER(object);

    auto rigidBody = btRigidBody::upcast(object->handle);
    if(!rigidBody)
        return APHY_INVALID_PARAMETER;

    world->handle->addRigidBody(rigidBody, collision_filter_group, collision_filter_mask);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyStepSimulation ( aphy_world* world, aphy_scalar time_step, aphy_int max_sub_steps, aphy_scalar fixed_time_step )
{
    CHECK_POINTER(world);
    world->handle->stepSimulation(time_step, max_sub_steps, fixed_time_step);
    return APHY_OK;
}
