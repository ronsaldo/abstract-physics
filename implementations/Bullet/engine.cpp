#include <stdio.h>
#include "engine.hpp"
#include "broadphase.hpp"
#include "collision_configuration.hpp"
#include "collision_dispatcher.hpp"
#include "collision_object.hpp"
#include "collision_shape.hpp"
#include "constraint_solver.hpp"
#include "motion_state.hpp"
#include "world.hpp"
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

inline int mapHeightUpAxis(aphy_axis axis)
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
_aphy_engine::_aphy_engine()
{
}

void _aphy_engine::lostReferences()
{
}

aphy_cstring aphy_engine::getName (  )
{
    return "Bullet";
}

aphy_int aphy_engine::getVersion (  )
{
    return 10;
}

APHY_EXPORT aphy_error aphyGetEngines ( aphy_size numengines, aphy_engine** engines, aphy_size* ret_numengines )
{
    if(ret_numengines)
        *ret_numengines = 1;

    if(engines && numengines >= 1)
        engines[0] = new aphy_engine();

    return APHY_OK;
}

// The exported C interface
APHY_EXPORT aphy_error aphyAddEngineReference ( aphy_engine* engine )
{
    CHECK_POINTER(engine);
    return engine->retain();
}

APHY_EXPORT aphy_error aphyReleaseEngine ( aphy_engine* engine )
{
    CHECK_POINTER(engine);
    return engine->release();
}

APHY_EXPORT aphy_cstring aphyGetEngineName ( aphy_engine* engine )
{
    if(!engine)
        return nullptr;
    return engine->getName();
}

APHY_EXPORT aphy_int aphyGetEngineVersion ( aphy_engine* engine )
{
    if(!engine)
        return 0;
    return engine->getVersion();
}

APHY_EXPORT aphy_collision_configuration* aphyCreateDefaultCollisionConfiguration ( aphy_engine* engine )
{
    if(!engine)
        return nullptr;

    return new aphy_collision_configuration(new btDefaultCollisionConfiguration());
}

APHY_EXPORT aphy_collision_dispatcher* aphyCreateDefaultCollisionDispatcher ( aphy_engine* engine, aphy_collision_configuration* collision_configuration )
{
    if(!engine)
        return nullptr;
    if(!collision_configuration)
        return nullptr;

    return new aphy_collision_dispatcher(new btCollisionDispatcher(collision_configuration->handle), collision_configuration);
}

APHY_EXPORT aphy_broadphase* aphyCreateDefaultBroadphase ( aphy_engine* engine )
{
    if(!engine)
        return nullptr;

    return new aphy_broadphase(new btDbvtBroadphase());
}

APHY_EXPORT aphy_constraint_solver* aphyCreateDefaultConstraintSolver ( aphy_engine* engine )
{
    if(!engine)
        return nullptr;

    return new aphy_constraint_solver(new btSequentialImpulseConstraintSolver());
}

APHY_EXPORT aphy_motion_state* aphyCreateDefaultMotionStte ( aphy_engine* engine )
{
    if(!engine)
        return nullptr;

    return new aphy_motion_state(new btDefaultMotionState());
}

APHY_EXPORT aphy_world* aphyCreateDynamicsWorld ( aphy_engine* engine, aphy_collision_dispatcher* collision_dispatcher, aphy_broadphase* broadphase, aphy_constraint_solver* constraint_solver, aphy_collision_configuration* collision_configuration )
{
    if(!engine)
        return nullptr;
    if(!collision_dispatcher)
        return nullptr;
    if(!constraint_solver)
        return nullptr;
    if(!collision_configuration)
        return nullptr;

    auto result = new aphy_world(
        new btDiscreteDynamicsWorld(collision_dispatcher->handle,
            broadphase->handle,
            constraint_solver->handle,
            collision_configuration->handle));

    collision_dispatcher->retain();     result->collisionDispatcher = collision_dispatcher;
    broadphase->retain();               result->broadphase = broadphase;
    constraint_solver->retain();        result->constraintSolver = constraint_solver;
    collision_configuration->retain();  result->collisionConfiguration = collision_configuration;
    return result;
}


APHY_EXPORT aphy_collision_shape* aphyCreateBoxShape ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth )
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btBoxShape(btVector3(half_width, half_height, half_depth)));
}

APHY_EXPORT aphy_collision_shape* aphyCreateConvexHullShape ( aphy_engine* engine, aphy_scalar* coordinates, aphy_size num_points, aphy_size stride )
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btConvexHullShape(coordinates, num_points, stride));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeX ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height)
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btCapsuleShapeX(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeY ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height)
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btCapsuleShape(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeZ ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height)
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btCapsuleShapeZ(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCompoundShape ( aphy_engine* engine )
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btCompoundShape(), APhyBulletCollisionShapeType::Compound);
}

APHY_EXPORT aphy_collision_shape* aphyCreateConeX ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height)
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btConeShapeX(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateConeY ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height)
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btConeShape(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateConeZ ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height)
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btConeShapeZ(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCylinderX ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth )
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btCylinderShapeX(btVector3(half_width, half_height, half_depth)));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCylinderY ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth )
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btCylinderShape(btVector3(half_width, half_height, half_depth)));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCylinderZ ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth )
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btCylinderShapeZ(btVector3(half_width, half_height, half_depth)));
}

APHY_EXPORT aphy_collision_shape* aphyCreateHeightfieldTerrainShape ( aphy_engine* engine,
        aphy_int height_stick_width, aphy_int height_stick_length, aphy_pointer heightfield_data,
        aphy_scalar height_scale, aphy_scalar min_height, aphy_scalar max_height, aphy_axis up_axis,
        aphy_scalar_type height_data_type, aphy_bool flip_quad_edges,
        aphy_scalar local_scale_x, aphy_scalar local_scale_y, aphy_scalar local_scale_z )
{
    if(!engine)
        return nullptr;
    auto shape = new btHeightfieldTerrainShape(height_stick_width, height_stick_length, heightfield_data,
        height_scale, min_height, max_height, mapHeightUpAxis(up_axis),
        mapHeightDataType(height_data_type), flip_quad_edges);

    shape->setLocalScaling(btVector3(local_scale_x, local_scale_y, local_scale_z));
    return new aphy_collision_shape(shape, APhyBulletCollisionShapeType::HeightField);
}

APHY_EXPORT aphy_collision_shape* aphyCreateSphere ( aphy_engine* engine, aphy_scalar radius )
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btSphereShape(radius));
}

APHY_EXPORT aphy_collision_shape* aphyCreateEmptyShape ( aphy_engine* engine )
{
    if(!engine)
        return nullptr;
    return new aphy_collision_shape(new btEmptyShape());
}

APHY_EXPORT aphy_collision_object* aphyCreateSimpleRigidBody ( aphy_engine* engine, aphy_scalar mass, aphy_motion_state* motion_state, aphy_collision_shape* collision_shape, aphy_vector3 local_inertia )
{
    if(!engine || !motion_state || !collision_shape)
        return nullptr;

    auto inertia = btVector3(local_inertia.x, local_inertia.y, local_inertia.z);
    auto result = new aphy_collision_object(new btRigidBody(mass, motion_state->handle, collision_shape->handle, inertia), true);
    motion_state->retain();     result->motionState = motion_state;
    collision_shape->retain();  result->collisionShape = collision_shape;
    return result;
}

APHY_EXPORT aphy_collision_object* aphyCreateSimpleRigidBodyFrom ( aphy_engine* engine, aphy_scalar mass, aphy_motion_state* motion_state, aphy_collision_shape* collision_shape, aphy_vector3 *local_inertia )
{
    if(!local_inertia)
        return nullptr;

    return aphyCreateSimpleRigidBody(engine, mass, motion_state, collision_shape, *local_inertia);
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
