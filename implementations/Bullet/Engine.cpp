#include <stdio.h>
#include "Engine.hpp"
#include "Broadphase.hpp"
#include "CollisionConfiguration.hpp"
#include "CollisionDispatcher.hpp"
#include "CollisionObject.hpp"
#include "CollisionShape.hpp"
#include "ConstraintSolver.hpp"
#include "MotionState.hpp"
#include "World.hpp"
#include "CharacterController.hpp"
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

APHY_EXPORT aphy_error aphyGetEngines ( aphy_size numengines, aphy_engine** engines, aphy_size* ret_numengines )
{
    using namespace aphy;
    using namespace APhyBullet;

    if(ret_numengines)
        *ret_numengines = 1;

    if(engines && numengines >= 1)
        engines[0] = reinterpret_cast<aphy_engine* > (makeObject<BulletEngine> ().disown());

    return APHY_OK;
}

namespace APhyBullet
{

inline int mapAxis(aphy_axis axis)
{
    switch(axis)
    {
    case APHY_AXIS_X: return 0;
    case APHY_AXIS_Y: return 1;
    case APHY_AXIS_Z:
    default:
        return 2;
    }
}

inline PHY_ScalarType mapHeightDataType(aphy_scalar_type type)
{
    switch(type)
    {
    default:
    case APHY_SCALAR_TYPE_UCHAR: return PHY_UCHAR;
    case APHY_SCALAR_TYPE_SHORT: return PHY_SHORT;
    case APHY_SCALAR_TYPE_FLOAT: return PHY_FLOAT;
    }
}

BulletEngine::BulletEngine()
{
}

BulletEngine::~BulletEngine()
{
}

aphy_cstring BulletEngine::getName (  )
{
    return "Bullet";
}

aphy_int BulletEngine::getVersion (  )
{
    return 10;
}

collision_configuration_ptr BulletEngine::createDefaultCollisionConfiguration()
{
    return makeObject<BulletCollisionConfiguration> (new btDefaultCollisionConfiguration ()).disown();
}

collision_dispatcher_ptr BulletEngine::createDefaultCollisionDispatcher(const collision_configuration_ref &collision_configuration)
{
    if(!collision_configuration)
        return nullptr;

    return makeObject<BulletCollisionDispatcher> (
            new btCollisionDispatcher(collision_configuration.as<BulletCollisionConfiguration> ()->handle),
            collision_configuration).disown();

}

broadphase_ptr BulletEngine::createDefaultBroadphase()
{
    auto handle = new btDbvtBroadphase();
    auto ghostPairCallback = new btGhostPairCallback();
    handle->getOverlappingPairCache()->setInternalGhostPairCallback(ghostPairCallback);
    return makeObject<BulletBroadphase> (handle, ghostPairCallback).disown();
}

constraint_solver_ptr BulletEngine::createDefaultConstraintSolver()
{
    return makeObject<BulletConstraintSolver> (new btSequentialImpulseConstraintSolver()).disown();
}

motion_state_ptr BulletEngine::createDefaultMotionState()
{
    return makeObject<BulletMotionState> (new btDefaultMotionState()).disown();
}

world_ptr BulletEngine::createDynamicsWorld(const collision_dispatcher_ref &collision_dispatcher, const broadphase_ref &broadphase, const constraint_solver_ref &constraint_solver, const collision_configuration_ref &collision_configuration)
{
    if(!collision_dispatcher)
        return nullptr;
    if(!constraint_solver)
        return nullptr;
    if(!collision_configuration)
        return nullptr;

    auto result = makeObject<BulletWorld> (
        new btDiscreteDynamicsWorld(collision_dispatcher.as<BulletCollisionDispatcher>()->handle,
            broadphase.as<BulletBroadphase>()->handle,
            constraint_solver.as<BulletConstraintSolver>()->handle,
            collision_configuration.as<BulletCollisionConfiguration>()->handle));

    auto world = result.as<BulletWorld> ();
    world->collisionDispatcher = collision_dispatcher;
    world->broadphase = broadphase;
    world->constraintSolver = constraint_solver;
    world->collisionConfiguration = collision_configuration;
    return result.disown();
}

collision_shape_ptr BulletEngine::createBoxShape(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth)
{
    return makeObject<BulletCollisionShape> (
        new btBoxShape(btVector3(half_width, half_height, half_depth))
    ).disown();
}

collision_shape_ptr BulletEngine::createCylinderX(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth)
{
    return makeObject<BulletCollisionShape> (
        new btCylinderShapeX(btVector3(half_width, half_height, half_depth))
    ).disown();
}

collision_shape_ptr BulletEngine::createCylinderY(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth)
{
    return makeObject<BulletCollisionShape> (
        new btCylinderShape(btVector3(half_width, half_height, half_depth))
    ).disown();
}

collision_shape_ptr BulletEngine::createCylinderZ(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth)
{
    return makeObject<BulletCollisionShape> (
        new btCylinderShapeZ(btVector3(half_width, half_height, half_depth))
    ).disown();
}

collision_shape_ptr BulletEngine::createConvexHullShape(aphy_scalar* coordinates, aphy_size num_points, aphy_size stride)
{
    return makeObject<BulletCollisionShape> (
        new btConvexHullShape(coordinates, num_points, stride)
    ).disown();
}

collision_shape_ptr BulletEngine::createCapsuleShapeX(aphy_scalar radius, aphy_scalar height)
{
    return makeObject<BulletCollisionShape> (
        new btCapsuleShapeX(radius, height)
    ).disown();
}

collision_shape_ptr BulletEngine::createCapsuleShapeY(aphy_scalar radius, aphy_scalar height)
{
    return makeObject<BulletCollisionShape> (
        new btCapsuleShape(radius, height)
    ).disown();
}

collision_shape_ptr BulletEngine::createCapsuleShapeZ(aphy_scalar radius, aphy_scalar height)
{
    return makeObject<BulletCollisionShape> (
        new btCapsuleShapeZ(radius, height)
    ).disown();
}

collision_shape_ptr BulletEngine::createCompoundShape()
{
    return makeObject<BulletCollisionShape> (
        new btCompoundShape(),
        APhyBulletCollisionShapeType::Compound
    ).disown();
}

collision_shape_ptr BulletEngine::createConeX(aphy_scalar radius, aphy_scalar height)
{
    return makeObject<BulletCollisionShape> (
        new btConeShapeX(radius, height)
    ).disown();
}

collision_shape_ptr BulletEngine::createConeY(aphy_scalar radius, aphy_scalar height)
{
    return makeObject<BulletCollisionShape> (
        new btConeShape(radius, height)
    ).disown();
}

collision_shape_ptr BulletEngine::createConeZ(aphy_scalar radius, aphy_scalar height)
{
    return makeObject<BulletCollisionShape> (
        new btConeShapeZ(radius, height)
    ).disown();
}

collision_shape_ptr BulletEngine::createEmptyShape()
{
    return makeObject<BulletCollisionShape> (
        new btEmptyShape()
    ).disown();
}

collision_shape_ptr BulletEngine::createHeightfieldTerrainShape(aphy_int height_stick_width, aphy_int height_stick_length, aphy_pointer heightfield_data, aphy_scalar height_scale, aphy_scalar min_height, aphy_scalar max_height, aphy_axis up_axis, aphy_scalar_type height_data_type, aphy_bool flip_quad_edges, aphy_scalar local_scale_x, aphy_scalar local_scale_y, aphy_scalar local_scale_z)
{
    auto shape = new btHeightfieldTerrainShape(height_stick_width, height_stick_length, heightfield_data,
        height_scale, min_height, max_height, mapAxis(up_axis),
        mapHeightDataType(height_data_type), flip_quad_edges);

    shape->setLocalScaling(btVector3(local_scale_x, local_scale_y, local_scale_z));

    return makeObject<BulletCollisionShape> (
        shape,
        APhyBulletCollisionShapeType::HeightField
    ).disown();
}

collision_shape_ptr BulletEngine::createSphere(aphy_scalar radius)
{
    return makeObject<BulletCollisionShape> (
        new btSphereShape(radius)
    ).disown();
}

collision_object_ptr BulletEngine::createSimpleRigidBody(aphy_scalar mass, const motion_state_ref &motion_state, const collision_shape_ref &collision_shape, aphy_vector3 local_inertia)
{
    if(!motion_state || !collision_shape)
        return nullptr;

    auto inertia = btVector3(local_inertia.x, local_inertia.y, local_inertia.z);
    auto result = makeObject<BulletCollisionObject> (
        new btRigidBody(mass,
            motion_state.as<BulletMotionState> ()->handle,
            collision_shape.as<BulletCollisionShape> ()->handle,
            inertia),
        APhyCollisionObjectType::RigidBody);

    auto collisionObject = result.as<BulletCollisionObject> ();
    collisionObject->motionState = motion_state;
    collisionObject->collisionShape = collision_shape;
    return result.disown();
}

collision_object_ptr BulletEngine::createSimpleRigidBodyFrom(aphy_scalar mass, const motion_state_ref &motion_state, const collision_shape_ref &collision_shape, aphy_vector3* local_inertia)
{
    if(!local_inertia)
        return nullptr;

    return createSimpleRigidBody(mass, motion_state, collision_shape, *local_inertia);
}

collision_object_ptr BulletEngine::createGhostObject()
{
    return makeObject<BulletCollisionObject> (new btGhostObject(), APhyCollisionObjectType::GhostObject).disown();
}

collision_object_ptr BulletEngine::createPairCachingGhostObject()
{
    return makeObject<BulletCollisionObject> (new btPairCachingGhostObject(), APhyCollisionObjectType::PairCachingGhostObject).disown();
}

character_controller_ptr BulletEngine::createKinematicCharacterController(const collision_object_ref &ghost_object, const collision_shape_ref &convex_shape, aphy_scalar step_height, aphy_axis up_axis)
{
    if(!ghost_object || ghost_object.as<BulletCollisionObject>()->type != APhyCollisionObjectType::PairCachingGhostObject)
        return nullptr;
    if(!convex_shape)
        return nullptr;

    auto handle = new btKinematicCharacterController (
            static_cast<btPairCachingGhostObject*> (ghost_object.as<BulletCollisionObject> ()->handle),
            static_cast<btConvexShape*> (convex_shape.as<BulletCollisionShape> ()->handle),
            step_height, mapAxis(up_axis));
    //handle->setUseGhostSweepTest(false);
    //handle->setMaxSlope(btRadians(70.0));

    auto result = makeObject<BulletCharacterController> (handle);
    auto character = result.as<BulletCharacterController> ();
    character->collisionObject = ghost_object;
    character->collisionShape = convex_shape;
    return result.disown();
}


#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

void printError(const char *format, ...)
{
    char buffer[2048];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, 2048, format, args);
#ifdef _WIN32
    if(!GetConsoleCP())
        OutputDebugStringA(buffer);
    else
        fputs(buffer, stderr);
#else
    fputs(buffer, stderr);
#endif
    va_end(args);
}

} // End of namespace APhyBullet
