
#ifndef _APHY_H_
#define _APHY_H_

#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef _WIN32
#   ifdef APHY_BUILD
#       define APHY_EXPORT __declspec(dllexport)
#   else
#       define APHY_EXPORT __declspec(dllimport)
#   endif
#else
#   if __GNUC__ >= 4
#       define APHY_EXPORT __attribute__ ((visibility ("default")))
#   endif
#endif

typedef unsigned char aphy_byte;
typedef signed char aphy_sbyte;
typedef signed short aphy_short;
typedef unsigned short aphy_ushort;
typedef signed int aphy_int;
typedef unsigned int aphy_uint;
typedef void* aphy_pointer;
typedef unsigned int aphy_size;
typedef int aphy_enum;
typedef int aphy_bool;
typedef float aphy_scalar;
typedef unsigned int aphy_bitfield;
typedef const char* aphy_cstring;
typedef const char* aphy_string;
typedef int aphy_string_length;
typedef char* aphy_cstring_buffer;
typedef char* aphy_string_buffer;

typedef struct _aphy_engine aphy_engine;
typedef struct _aphy_collision_configuration aphy_collision_configuration;
typedef struct _aphy_collision_dispatcher aphy_collision_dispatcher;
typedef struct _aphy_broadphase aphy_broadphase;
typedef struct _aphy_constraint_solver aphy_constraint_solver;
typedef struct _aphy_world aphy_world;
typedef struct _aphy_collision_object aphy_collision_object;
typedef struct _aphy_collision_shape aphy_collision_shape;

typedef enum {
	APHY_OK = 0,
	APHY_ERROR = -1,
	APHY_NULL_POINTER = -2,
	APHY_INVALID_OPERATION = -3,
	APHY_INVALID_PARAMETER = -4,
	APHY_OUT_OF_BOUNDS = -5,
	APHY_UNSUPPORTED = -6,
	APHY_UNIMPLEMENTED = -7,
} aphy_error;


/* Global functions. */
typedef aphy_error (*aphyGetEngines_FUN) ( aphy_size numengines, aphy_engine** engines, aphy_size* ret_numengines );

APHY_EXPORT aphy_error aphyGetEngines ( aphy_size numengines, aphy_engine** engines, aphy_size* ret_numengines );

/* Methods for interface aphy_engine. */
typedef aphy_error (*aphyAddEngineReference_FUN) ( aphy_engine* engine );
typedef aphy_error (*aphyReleaseEngine_FUN) ( aphy_engine* engine );
typedef aphy_cstring (*aphyGetEngineName_FUN) ( aphy_engine* engine );
typedef aphy_int (*aphyGetEngineVersion_FUN) ( aphy_engine* engine );
typedef aphy_collision_configuration* (*aphyCreateDefaultCollisionConfiguration_FUN) ( aphy_engine* engine );
typedef aphy_collision_dispatcher* (*aphyCreateDefaultCollisionDispatcher_FUN) ( aphy_engine* engine, aphy_collision_configuration* collision_configuration );
typedef aphy_broadphase* (*aphyCreateDefaultBroadphase_FUN) ( aphy_engine* engine );
typedef aphy_constraint_solver* (*aphyCreateDefaultConstraintSolver_FUN) ( aphy_engine* engine );
typedef aphy_world* (*aphyCreateDynamicsWorld_FUN) ( aphy_engine* engine, aphy_collision_dispatcher* collision_dispatcher, aphy_broadphase* broadphase, aphy_constraint_solver* constraint_solver, aphy_collision_configuration* collision_configuration );
typedef aphy_collision_shape* (*aphyCreateBoxShape_FUN) ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth );
typedef aphy_collision_shape* (*aphyCreateCylinderX_FUN) ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth );
typedef aphy_collision_shape* (*aphyCreateCylinderY_FUN) ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth );
typedef aphy_collision_shape* (*aphyCreateCylinderZ_FUN) ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth );
typedef aphy_collision_shape* (*aphyCreateConvexHullShape_FUN) ( aphy_engine* engine, aphy_scalar* coordinates, aphy_size num_points, aphy_size stride );
typedef aphy_collision_shape* (*aphyCreateCapsuleShapeX_FUN) ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
typedef aphy_collision_shape* (*aphyCreateCapsuleShapeY_FUN) ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
typedef aphy_collision_shape* (*aphyCreateCapsuleShapeZ_FUN) ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
typedef aphy_collision_shape* (*aphyCreateConeX_FUN) ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
typedef aphy_collision_shape* (*aphyCreateConeY_FUN) ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
typedef aphy_collision_shape* (*aphyCreateConeZ_FUN) ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
typedef aphy_collision_shape* (*aphyCreateSphere_FUN) ( aphy_engine* engine, aphy_scalar radius );

APHY_EXPORT aphy_error aphyAddEngineReference ( aphy_engine* engine );
APHY_EXPORT aphy_error aphyReleaseEngine ( aphy_engine* engine );
APHY_EXPORT aphy_cstring aphyGetEngineName ( aphy_engine* engine );
APHY_EXPORT aphy_int aphyGetEngineVersion ( aphy_engine* engine );
APHY_EXPORT aphy_collision_configuration* aphyCreateDefaultCollisionConfiguration ( aphy_engine* engine );
APHY_EXPORT aphy_collision_dispatcher* aphyCreateDefaultCollisionDispatcher ( aphy_engine* engine, aphy_collision_configuration* collision_configuration );
APHY_EXPORT aphy_broadphase* aphyCreateDefaultBroadphase ( aphy_engine* engine );
APHY_EXPORT aphy_constraint_solver* aphyCreateDefaultConstraintSolver ( aphy_engine* engine );
APHY_EXPORT aphy_world* aphyCreateDynamicsWorld ( aphy_engine* engine, aphy_collision_dispatcher* collision_dispatcher, aphy_broadphase* broadphase, aphy_constraint_solver* constraint_solver, aphy_collision_configuration* collision_configuration );
APHY_EXPORT aphy_collision_shape* aphyCreateBoxShape ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth );
APHY_EXPORT aphy_collision_shape* aphyCreateCylinderX ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth );
APHY_EXPORT aphy_collision_shape* aphyCreateCylinderY ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth );
APHY_EXPORT aphy_collision_shape* aphyCreateCylinderZ ( aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth );
APHY_EXPORT aphy_collision_shape* aphyCreateConvexHullShape ( aphy_engine* engine, aphy_scalar* coordinates, aphy_size num_points, aphy_size stride );
APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeX ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeY ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeZ ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
APHY_EXPORT aphy_collision_shape* aphyCreateConeX ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
APHY_EXPORT aphy_collision_shape* aphyCreateConeY ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
APHY_EXPORT aphy_collision_shape* aphyCreateConeZ ( aphy_engine* engine, aphy_scalar radius, aphy_scalar height );
APHY_EXPORT aphy_collision_shape* aphyCreateSphere ( aphy_engine* engine, aphy_scalar radius );

/* Methods for interface aphy_collision_configuration. */
typedef aphy_error (*aphyAddCollisionConfigurationReference_FUN) ( aphy_collision_configuration* collision_configuration );
typedef aphy_error (*aphyReleaseCollisionConfiguration_FUN) ( aphy_collision_configuration* collision_configuration );

APHY_EXPORT aphy_error aphyAddCollisionConfigurationReference ( aphy_collision_configuration* collision_configuration );
APHY_EXPORT aphy_error aphyReleaseCollisionConfiguration ( aphy_collision_configuration* collision_configuration );

/* Methods for interface aphy_collision_dispatcher. */
typedef aphy_error (*aphyAddCollisionDispatcherReference_FUN) ( aphy_collision_dispatcher* collision_dispatcher );
typedef aphy_error (*aphyReleaseCollisionDispatcher_FUN) ( aphy_collision_dispatcher* collision_dispatcher );

APHY_EXPORT aphy_error aphyAddCollisionDispatcherReference ( aphy_collision_dispatcher* collision_dispatcher );
APHY_EXPORT aphy_error aphyReleaseCollisionDispatcher ( aphy_collision_dispatcher* collision_dispatcher );

/* Methods for interface aphy_broadphase. */
typedef aphy_error (*aphyAddBroadphaseReference_FUN) ( aphy_broadphase* broadphase );
typedef aphy_error (*aphyReleaseBroadphaseReference_FUN) ( aphy_broadphase* broadphase );

APHY_EXPORT aphy_error aphyAddBroadphaseReference ( aphy_broadphase* broadphase );
APHY_EXPORT aphy_error aphyReleaseBroadphaseReference ( aphy_broadphase* broadphase );

/* Methods for interface aphy_constraint_solver. */
typedef aphy_error (*aphyAddConstraintSolverReference_FUN) ( aphy_constraint_solver* constraint_solver );
typedef aphy_error (*aphyReleaseConstraintSolverReference_FUN) ( aphy_constraint_solver* constraint_solver );

APHY_EXPORT aphy_error aphyAddConstraintSolverReference ( aphy_constraint_solver* constraint_solver );
APHY_EXPORT aphy_error aphyReleaseConstraintSolverReference ( aphy_constraint_solver* constraint_solver );

/* Methods for interface aphy_world. */
typedef aphy_error (*aphyAddWorldReference_FUN) ( aphy_world* world );
typedef aphy_error (*aphyReleaseWorldReference_FUN) ( aphy_world* world );
typedef aphy_uint (*aphyGetNumberOfCollisionObject_FUN) ( aphy_world* world );
typedef aphy_uint (*aphyGetNumberOfConstraints_FUN) ( aphy_world* world );
typedef aphy_error (*aphyAddCollisionObject_FUN) ( aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask );
typedef aphy_error (*aphyRemoveCollisionObject_FUN) ( aphy_world* world, aphy_collision_object* object );
typedef aphy_error (*aphyAddRigidBody_FUN) ( aphy_world* world, aphy_collision_object* object );
typedef aphy_error (*aphyRemoveRigidBody_FUN) ( aphy_world* world, aphy_collision_object* object );
typedef aphy_error (*aphyAddRigidBodyWithFilter_FUN) ( aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask );
typedef aphy_error (*aphyStepSimulation_FUN) ( aphy_world* world, aphy_scalar time_step, aphy_int max_sub_steps, aphy_scalar fixed_time_step );
typedef aphy_error (*aphySetGravity_FUN) ( aphy_world* world, aphy_scalar x, aphy_scalar y, aphy_scalar z );

APHY_EXPORT aphy_error aphyAddWorldReference ( aphy_world* world );
APHY_EXPORT aphy_error aphyReleaseWorldReference ( aphy_world* world );
APHY_EXPORT aphy_uint aphyGetNumberOfCollisionObject ( aphy_world* world );
APHY_EXPORT aphy_uint aphyGetNumberOfConstraints ( aphy_world* world );
APHY_EXPORT aphy_error aphyAddCollisionObject ( aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask );
APHY_EXPORT aphy_error aphyRemoveCollisionObject ( aphy_world* world, aphy_collision_object* object );
APHY_EXPORT aphy_error aphyAddRigidBody ( aphy_world* world, aphy_collision_object* object );
APHY_EXPORT aphy_error aphyRemoveRigidBody ( aphy_world* world, aphy_collision_object* object );
APHY_EXPORT aphy_error aphyAddRigidBodyWithFilter ( aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask );
APHY_EXPORT aphy_error aphyStepSimulation ( aphy_world* world, aphy_scalar time_step, aphy_int max_sub_steps, aphy_scalar fixed_time_step );
APHY_EXPORT aphy_error aphySetGravity ( aphy_world* world, aphy_scalar x, aphy_scalar y, aphy_scalar z );

/* Methods for interface aphy_collision_object. */
typedef aphy_error (*aphyAddCollisionObjectReference_FUN) ( aphy_collision_object* collision_object );
typedef aphy_error (*aphyReleaseCollisionObjectReference_FUN) ( aphy_collision_object* collision_object );

APHY_EXPORT aphy_error aphyAddCollisionObjectReference ( aphy_collision_object* collision_object );
APHY_EXPORT aphy_error aphyReleaseCollisionObjectReference ( aphy_collision_object* collision_object );

/* Methods for interface aphy_collision_shape. */
typedef aphy_error (*aphyAddCollisionShapeReference_FUN) ( aphy_collision_shape* collision_shape );
typedef aphy_error (*aphyReleaseCollisionShapeReference_FUN) ( aphy_collision_shape* collision_shape );
typedef aphy_error (*aphySetShapeMargin_FUN) ( aphy_collision_shape* collision_shape, aphy_scalar margin );
typedef aphy_scalar (*aphyGetShapeMargin_FUN) ( aphy_collision_shape* collision_shape );

APHY_EXPORT aphy_error aphyAddCollisionShapeReference ( aphy_collision_shape* collision_shape );
APHY_EXPORT aphy_error aphyReleaseCollisionShapeReference ( aphy_collision_shape* collision_shape );
APHY_EXPORT aphy_error aphySetShapeMargin ( aphy_collision_shape* collision_shape, aphy_scalar margin );
APHY_EXPORT aphy_scalar aphyGetShapeMargin ( aphy_collision_shape* collision_shape );

/* Installable client driver interface. */
typedef struct _aphy_icd_dispatch {
	int icd_interface_version;
	aphyGetEngines_FUN aphyGetEngines;
	aphyAddEngineReference_FUN aphyAddEngineReference;
	aphyReleaseEngine_FUN aphyReleaseEngine;
	aphyGetEngineName_FUN aphyGetEngineName;
	aphyGetEngineVersion_FUN aphyGetEngineVersion;
	aphyCreateDefaultCollisionConfiguration_FUN aphyCreateDefaultCollisionConfiguration;
	aphyCreateDefaultCollisionDispatcher_FUN aphyCreateDefaultCollisionDispatcher;
	aphyCreateDefaultBroadphase_FUN aphyCreateDefaultBroadphase;
	aphyCreateDefaultConstraintSolver_FUN aphyCreateDefaultConstraintSolver;
	aphyCreateDynamicsWorld_FUN aphyCreateDynamicsWorld;
	aphyCreateBoxShape_FUN aphyCreateBoxShape;
	aphyCreateCylinderX_FUN aphyCreateCylinderX;
	aphyCreateCylinderY_FUN aphyCreateCylinderY;
	aphyCreateCylinderZ_FUN aphyCreateCylinderZ;
	aphyCreateConvexHullShape_FUN aphyCreateConvexHullShape;
	aphyCreateCapsuleShapeX_FUN aphyCreateCapsuleShapeX;
	aphyCreateCapsuleShapeY_FUN aphyCreateCapsuleShapeY;
	aphyCreateCapsuleShapeZ_FUN aphyCreateCapsuleShapeZ;
	aphyCreateConeX_FUN aphyCreateConeX;
	aphyCreateConeY_FUN aphyCreateConeY;
	aphyCreateConeZ_FUN aphyCreateConeZ;
	aphyCreateSphere_FUN aphyCreateSphere;
	aphyAddCollisionConfigurationReference_FUN aphyAddCollisionConfigurationReference;
	aphyReleaseCollisionConfiguration_FUN aphyReleaseCollisionConfiguration;
	aphyAddCollisionDispatcherReference_FUN aphyAddCollisionDispatcherReference;
	aphyReleaseCollisionDispatcher_FUN aphyReleaseCollisionDispatcher;
	aphyAddBroadphaseReference_FUN aphyAddBroadphaseReference;
	aphyReleaseBroadphaseReference_FUN aphyReleaseBroadphaseReference;
	aphyAddConstraintSolverReference_FUN aphyAddConstraintSolverReference;
	aphyReleaseConstraintSolverReference_FUN aphyReleaseConstraintSolverReference;
	aphyAddWorldReference_FUN aphyAddWorldReference;
	aphyReleaseWorldReference_FUN aphyReleaseWorldReference;
	aphyGetNumberOfCollisionObject_FUN aphyGetNumberOfCollisionObject;
	aphyGetNumberOfConstraints_FUN aphyGetNumberOfConstraints;
	aphyAddCollisionObject_FUN aphyAddCollisionObject;
	aphyRemoveCollisionObject_FUN aphyRemoveCollisionObject;
	aphyAddRigidBody_FUN aphyAddRigidBody;
	aphyRemoveRigidBody_FUN aphyRemoveRigidBody;
	aphyAddRigidBodyWithFilter_FUN aphyAddRigidBodyWithFilter;
	aphyStepSimulation_FUN aphyStepSimulation;
	aphySetGravity_FUN aphySetGravity;
	aphyAddCollisionObjectReference_FUN aphyAddCollisionObjectReference;
	aphyReleaseCollisionObjectReference_FUN aphyReleaseCollisionObjectReference;
	aphyAddCollisionShapeReference_FUN aphyAddCollisionShapeReference;
	aphyReleaseCollisionShapeReference_FUN aphyReleaseCollisionShapeReference;
	aphySetShapeMargin_FUN aphySetShapeMargin;
	aphyGetShapeMargin_FUN aphyGetShapeMargin;
} aphy_icd_dispatch;


#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* _APHY_H_ */