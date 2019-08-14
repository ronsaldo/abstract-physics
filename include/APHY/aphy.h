
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
typedef struct _aphy_collision_mesh_collection aphy_collision_mesh_collection;
typedef struct _aphy_constraint_solver aphy_constraint_solver;
typedef struct _aphy_world aphy_world;
typedef struct _aphy_character_controller aphy_character_controller;
typedef struct _aphy_collision_object aphy_collision_object;
typedef struct _aphy_collision_shape aphy_collision_shape;
typedef struct _aphy_motion_state aphy_motion_state;

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

typedef enum {
	APHY_AXIS_X = 0,
	APHY_AXIS_Y = 1,
	APHY_AXIS_Z = 2,
} aphy_axis;

typedef enum {
	APHY_SCALAR_TYPE_UCHAR = 0,
	APHY_SCALAR_TYPE_SHORT = 1,
	APHY_SCALAR_TYPE_FLOAT = 2,
} aphy_scalar_type;

typedef enum {
	APHY_DEBUG_DRAW_OP_NOP = 0,
	APHY_DEBUG_DRAW_OP_LINE = 1,
	APHY_DEBUG_DRAW_OP_LINE_GRADIENT = 2,
	APHY_DEBUG_DRAW_OP_TRIANGLE_FLAT = 3,
	APHY_DEBUG_DRAW_OP_TRIANGLE_GRADIENT = 4,
	APHY_DEBUG_DRAW_OP_TRIANGLE_LIGHTED = 5,
	APHY_DEBUG_DRAW_OP_CONTACT_POINT = 6,
	APHY_DEBUG_DRAW_OP_ERROR_WARNING = 7,
	APHY_DEBUG_DRAW_OP_3DTEXT = 8,
} aphy_debug_draw_opcode;


/* Structure aphy_vector3. */
typedef struct aphy_vector3 {
	aphy_scalar x;
	aphy_scalar y;
	aphy_scalar z;
	aphy_scalar pad;
} aphy_vector3;

/* Structure aphy_quaternion. */
typedef struct aphy_quaternion {
	aphy_scalar x;
	aphy_scalar y;
	aphy_scalar z;
	aphy_scalar w;
} aphy_quaternion;

/* Structure aphy_matrix3x3. */
typedef struct aphy_matrix3x3 {
	aphy_vector3 firstRow;
	aphy_vector3 secondRow;
	aphy_vector3 thirdRow;
} aphy_matrix3x3;

/* Structure aphy_transform. */
typedef struct aphy_transform {
	aphy_matrix3x3 rotation;
	aphy_vector3 origin;
} aphy_transform;

/* Structure aphy_collision_mesh_accessor. */
typedef struct aphy_collision_mesh_accessor {
	aphy_pointer vertices;
	aphy_size vertex_offset;
	aphy_size vertex_stride;
	aphy_size vertex_count;
	aphy_pointer indices;
	aphy_size index_offset;
	aphy_size index_stride;
	aphy_size index_count;
} aphy_collision_mesh_accessor;

/* Global functions. */
typedef aphy_error (*aphyGetEngines_FUN) (aphy_size numengines, aphy_engine** engines, aphy_size* ret_numengines);

APHY_EXPORT aphy_error aphyGetEngines(aphy_size numengines, aphy_engine** engines, aphy_size* ret_numengines);

/* Methods for interface aphy_engine. */
typedef aphy_error (*aphyAddEngineReference_FUN) (aphy_engine* engine);
typedef aphy_error (*aphyReleaseEngine_FUN) (aphy_engine* engine);
typedef aphy_cstring (*aphyGetEngineName_FUN) (aphy_engine* engine);
typedef aphy_int (*aphyGetEngineVersion_FUN) (aphy_engine* engine);
typedef aphy_collision_configuration* (*aphyCreateDefaultCollisionConfiguration_FUN) (aphy_engine* engine);
typedef aphy_collision_dispatcher* (*aphyCreateDefaultCollisionDispatcher_FUN) (aphy_engine* engine, aphy_collision_configuration* collision_configuration);
typedef aphy_broadphase* (*aphyCreateDefaultBroadphase_FUN) (aphy_engine* engine);
typedef aphy_constraint_solver* (*aphyCreateDefaultConstraintSolver_FUN) (aphy_engine* engine);
typedef aphy_motion_state* (*aphyCreateDefaultMotionStte_FUN) (aphy_engine* engine);
typedef aphy_world* (*aphyCreateDynamicsWorld_FUN) (aphy_engine* engine, aphy_collision_dispatcher* collision_dispatcher, aphy_broadphase* broadphase, aphy_constraint_solver* constraint_solver, aphy_collision_configuration* collision_configuration);
typedef aphy_collision_shape* (*aphyCreateBoxShape_FUN) (aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth);
typedef aphy_collision_shape* (*aphyCreateCylinderX_FUN) (aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth);
typedef aphy_collision_shape* (*aphyCreateCylinderY_FUN) (aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth);
typedef aphy_collision_shape* (*aphyCreateCylinderZ_FUN) (aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth);
typedef aphy_collision_shape* (*aphyCreateConvexHullShape_FUN) (aphy_engine* engine, aphy_scalar* coordinates, aphy_size num_points, aphy_size stride);
typedef aphy_collision_shape* (*aphyCreateCapsuleShapeX_FUN) (aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
typedef aphy_collision_shape* (*aphyCreateCapsuleShapeY_FUN) (aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
typedef aphy_collision_shape* (*aphyCreateCapsuleShapeZ_FUN) (aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
typedef aphy_collision_shape* (*aphyCreateCompoundShape_FUN) (aphy_engine* engine);
typedef aphy_collision_shape* (*aphyCreateConeX_FUN) (aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
typedef aphy_collision_shape* (*aphyCreateConeY_FUN) (aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
typedef aphy_collision_shape* (*aphyCreateConeZ_FUN) (aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
typedef aphy_collision_shape* (*aphyCreateEmptyShape_FUN) (aphy_engine* engine);
typedef aphy_collision_shape* (*aphyCreateHeightfieldTerrainShape_FUN) (aphy_engine* engine, aphy_int height_stick_width, aphy_int height_stick_length, aphy_pointer heightfield_data, aphy_scalar height_scale, aphy_scalar min_height, aphy_scalar max_height, aphy_axis up_axis, aphy_scalar_type height_data_type, aphy_bool flip_quad_edges, aphy_scalar local_scale_x, aphy_scalar local_scale_y, aphy_scalar local_scale_z);
typedef aphy_collision_mesh_collection* (*aphyCreateCollisionMeshCollection_FUN) (aphy_engine* engine);
typedef aphy_collision_shape* (*aphyCreateTriangleMeshCollisionShape_FUN) (aphy_engine* engine, aphy_collision_mesh_collection* mesh_collection);
typedef aphy_collision_shape* (*aphyCreateSphere_FUN) (aphy_engine* engine, aphy_scalar radius);
typedef aphy_collision_object* (*aphyCreateSimpleRigidBody_FUN) (aphy_engine* engine, aphy_scalar mass, aphy_motion_state* motion_state, aphy_collision_shape* collision_shape, aphy_vector3 local_inertia);
typedef aphy_collision_object* (*aphyCreateSimpleRigidBodyFrom_FUN) (aphy_engine* engine, aphy_scalar mass, aphy_motion_state* motion_state, aphy_collision_shape* collision_shape, aphy_vector3* local_inertia);
typedef aphy_collision_object* (*aphyCreateGhostObject_FUN) (aphy_engine* engine);
typedef aphy_collision_object* (*aphyCreatePairCachingGhostObject_FUN) (aphy_engine* engine);
typedef aphy_character_controller* (*aphyCreateKinematicCharacterController_FUN) (aphy_engine* engine, aphy_collision_object* ghost_object, aphy_collision_shape* convex_shape, aphy_scalar step_height, aphy_axis up_axis);

APHY_EXPORT aphy_error aphyAddEngineReference(aphy_engine* engine);
APHY_EXPORT aphy_error aphyReleaseEngine(aphy_engine* engine);
APHY_EXPORT aphy_cstring aphyGetEngineName(aphy_engine* engine);
APHY_EXPORT aphy_int aphyGetEngineVersion(aphy_engine* engine);
APHY_EXPORT aphy_collision_configuration* aphyCreateDefaultCollisionConfiguration(aphy_engine* engine);
APHY_EXPORT aphy_collision_dispatcher* aphyCreateDefaultCollisionDispatcher(aphy_engine* engine, aphy_collision_configuration* collision_configuration);
APHY_EXPORT aphy_broadphase* aphyCreateDefaultBroadphase(aphy_engine* engine);
APHY_EXPORT aphy_constraint_solver* aphyCreateDefaultConstraintSolver(aphy_engine* engine);
APHY_EXPORT aphy_motion_state* aphyCreateDefaultMotionStte(aphy_engine* engine);
APHY_EXPORT aphy_world* aphyCreateDynamicsWorld(aphy_engine* engine, aphy_collision_dispatcher* collision_dispatcher, aphy_broadphase* broadphase, aphy_constraint_solver* constraint_solver, aphy_collision_configuration* collision_configuration);
APHY_EXPORT aphy_collision_shape* aphyCreateBoxShape(aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth);
APHY_EXPORT aphy_collision_shape* aphyCreateCylinderX(aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth);
APHY_EXPORT aphy_collision_shape* aphyCreateCylinderY(aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth);
APHY_EXPORT aphy_collision_shape* aphyCreateCylinderZ(aphy_engine* engine, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth);
APHY_EXPORT aphy_collision_shape* aphyCreateConvexHullShape(aphy_engine* engine, aphy_scalar* coordinates, aphy_size num_points, aphy_size stride);
APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeX(aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeY(aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeZ(aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
APHY_EXPORT aphy_collision_shape* aphyCreateCompoundShape(aphy_engine* engine);
APHY_EXPORT aphy_collision_shape* aphyCreateConeX(aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
APHY_EXPORT aphy_collision_shape* aphyCreateConeY(aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
APHY_EXPORT aphy_collision_shape* aphyCreateConeZ(aphy_engine* engine, aphy_scalar radius, aphy_scalar height);
APHY_EXPORT aphy_collision_shape* aphyCreateEmptyShape(aphy_engine* engine);
APHY_EXPORT aphy_collision_shape* aphyCreateHeightfieldTerrainShape(aphy_engine* engine, aphy_int height_stick_width, aphy_int height_stick_length, aphy_pointer heightfield_data, aphy_scalar height_scale, aphy_scalar min_height, aphy_scalar max_height, aphy_axis up_axis, aphy_scalar_type height_data_type, aphy_bool flip_quad_edges, aphy_scalar local_scale_x, aphy_scalar local_scale_y, aphy_scalar local_scale_z);
APHY_EXPORT aphy_collision_mesh_collection* aphyCreateCollisionMeshCollection(aphy_engine* engine);
APHY_EXPORT aphy_collision_shape* aphyCreateTriangleMeshCollisionShape(aphy_engine* engine, aphy_collision_mesh_collection* mesh_collection);
APHY_EXPORT aphy_collision_shape* aphyCreateSphere(aphy_engine* engine, aphy_scalar radius);
APHY_EXPORT aphy_collision_object* aphyCreateSimpleRigidBody(aphy_engine* engine, aphy_scalar mass, aphy_motion_state* motion_state, aphy_collision_shape* collision_shape, aphy_vector3 local_inertia);
APHY_EXPORT aphy_collision_object* aphyCreateSimpleRigidBodyFrom(aphy_engine* engine, aphy_scalar mass, aphy_motion_state* motion_state, aphy_collision_shape* collision_shape, aphy_vector3* local_inertia);
APHY_EXPORT aphy_collision_object* aphyCreateGhostObject(aphy_engine* engine);
APHY_EXPORT aphy_collision_object* aphyCreatePairCachingGhostObject(aphy_engine* engine);
APHY_EXPORT aphy_character_controller* aphyCreateKinematicCharacterController(aphy_engine* engine, aphy_collision_object* ghost_object, aphy_collision_shape* convex_shape, aphy_scalar step_height, aphy_axis up_axis);

/* Methods for interface aphy_collision_configuration. */
typedef aphy_error (*aphyAddCollisionConfigurationReference_FUN) (aphy_collision_configuration* collision_configuration);
typedef aphy_error (*aphyReleaseCollisionConfiguration_FUN) (aphy_collision_configuration* collision_configuration);

APHY_EXPORT aphy_error aphyAddCollisionConfigurationReference(aphy_collision_configuration* collision_configuration);
APHY_EXPORT aphy_error aphyReleaseCollisionConfiguration(aphy_collision_configuration* collision_configuration);

/* Methods for interface aphy_collision_dispatcher. */
typedef aphy_error (*aphyAddCollisionDispatcherReference_FUN) (aphy_collision_dispatcher* collision_dispatcher);
typedef aphy_error (*aphyReleaseCollisionDispatcher_FUN) (aphy_collision_dispatcher* collision_dispatcher);

APHY_EXPORT aphy_error aphyAddCollisionDispatcherReference(aphy_collision_dispatcher* collision_dispatcher);
APHY_EXPORT aphy_error aphyReleaseCollisionDispatcher(aphy_collision_dispatcher* collision_dispatcher);

/* Methods for interface aphy_broadphase. */
typedef aphy_error (*aphyAddBroadphaseReference_FUN) (aphy_broadphase* broadphase);
typedef aphy_error (*aphyReleaseBroadphaseReference_FUN) (aphy_broadphase* broadphase);

APHY_EXPORT aphy_error aphyAddBroadphaseReference(aphy_broadphase* broadphase);
APHY_EXPORT aphy_error aphyReleaseBroadphaseReference(aphy_broadphase* broadphase);

/* Methods for interface aphy_collision_mesh_collection. */
typedef aphy_error (*aphyAddCollisionMeshCollectionReference_FUN) (aphy_collision_mesh_collection* collision_mesh_collection);
typedef aphy_error (*aphyReleaseCollisionMeshCollectionReference_FUN) (aphy_collision_mesh_collection* collision_mesh_collection);
typedef aphy_error (*aphyAddCollisionMeshAccessorToCollection_FUN) (aphy_collision_mesh_collection* collision_mesh_collection, aphy_collision_mesh_accessor* accessor, aphy_transform* transform);

APHY_EXPORT aphy_error aphyAddCollisionMeshCollectionReference(aphy_collision_mesh_collection* collision_mesh_collection);
APHY_EXPORT aphy_error aphyReleaseCollisionMeshCollectionReference(aphy_collision_mesh_collection* collision_mesh_collection);
APHY_EXPORT aphy_error aphyAddCollisionMeshAccessorToCollection(aphy_collision_mesh_collection* collision_mesh_collection, aphy_collision_mesh_accessor* accessor, aphy_transform* transform);

/* Methods for interface aphy_constraint_solver. */
typedef aphy_error (*aphyAddConstraintSolverReference_FUN) (aphy_constraint_solver* constraint_solver);
typedef aphy_error (*aphyReleaseConstraintSolverReference_FUN) (aphy_constraint_solver* constraint_solver);

APHY_EXPORT aphy_error aphyAddConstraintSolverReference(aphy_constraint_solver* constraint_solver);
APHY_EXPORT aphy_error aphyReleaseConstraintSolverReference(aphy_constraint_solver* constraint_solver);

/* Methods for interface aphy_world. */
typedef aphy_error (*aphyAddWorldReference_FUN) (aphy_world* world);
typedef aphy_error (*aphyReleaseWorldReference_FUN) (aphy_world* world);
typedef aphy_uint (*aphyGetNumberOfCollisionObject_FUN) (aphy_world* world);
typedef aphy_uint (*aphyGetNumberOfConstraints_FUN) (aphy_world* world);
typedef aphy_error (*aphyAddCollisionObject_FUN) (aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask);
typedef aphy_error (*aphyRemoveCollisionObject_FUN) (aphy_world* world, aphy_collision_object* object);
typedef aphy_error (*aphyAddRigidBody_FUN) (aphy_world* world, aphy_collision_object* object);
typedef aphy_error (*aphyRemoveRigidBody_FUN) (aphy_world* world, aphy_collision_object* object);
typedef aphy_error (*aphyAddCharacterController_FUN) (aphy_world* world, aphy_character_controller* character);
typedef aphy_error (*aphyRemoveCharacterController_FUN) (aphy_world* world, aphy_character_controller* character);
typedef aphy_error (*aphyAddRigidBodyWithFilter_FUN) (aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask);
typedef aphy_error (*aphyStepSimulation_FUN) (aphy_world* world, aphy_scalar time_step, aphy_int max_sub_steps, aphy_scalar fixed_time_step);
typedef aphy_error (*aphySetGravity_FUN) (aphy_world* world, aphy_scalar x, aphy_scalar y, aphy_scalar z);
typedef aphy_size (*aphyEncodeDebugDrawing_FUN) (aphy_world* world);
typedef aphy_error (*aphyGetDebugDrawingData_FUN) (aphy_world* world, aphy_size buffer_size, aphy_pointer buffer);

APHY_EXPORT aphy_error aphyAddWorldReference(aphy_world* world);
APHY_EXPORT aphy_error aphyReleaseWorldReference(aphy_world* world);
APHY_EXPORT aphy_uint aphyGetNumberOfCollisionObject(aphy_world* world);
APHY_EXPORT aphy_uint aphyGetNumberOfConstraints(aphy_world* world);
APHY_EXPORT aphy_error aphyAddCollisionObject(aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask);
APHY_EXPORT aphy_error aphyRemoveCollisionObject(aphy_world* world, aphy_collision_object* object);
APHY_EXPORT aphy_error aphyAddRigidBody(aphy_world* world, aphy_collision_object* object);
APHY_EXPORT aphy_error aphyRemoveRigidBody(aphy_world* world, aphy_collision_object* object);
APHY_EXPORT aphy_error aphyAddCharacterController(aphy_world* world, aphy_character_controller* character);
APHY_EXPORT aphy_error aphyRemoveCharacterController(aphy_world* world, aphy_character_controller* character);
APHY_EXPORT aphy_error aphyAddRigidBodyWithFilter(aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask);
APHY_EXPORT aphy_error aphyStepSimulation(aphy_world* world, aphy_scalar time_step, aphy_int max_sub_steps, aphy_scalar fixed_time_step);
APHY_EXPORT aphy_error aphySetGravity(aphy_world* world, aphy_scalar x, aphy_scalar y, aphy_scalar z);
APHY_EXPORT aphy_size aphyEncodeDebugDrawing(aphy_world* world);
APHY_EXPORT aphy_error aphyGetDebugDrawingData(aphy_world* world, aphy_size buffer_size, aphy_pointer buffer);

/* Methods for interface aphy_character_controller. */
typedef aphy_error (*aphyAddCharacterControllerReference_FUN) (aphy_character_controller* character_controller);
typedef aphy_error (*aphyReleaseCharacterControllerReference_FUN) (aphy_character_controller* character_controller);
typedef aphy_error (*aphySetCharacterControllerWalkDirection_FUN) (aphy_character_controller* character_controller, aphy_vector3 direction);
typedef aphy_error (*aphySetCharacterControllerWalkDirectionFrom_FUN) (aphy_character_controller* character_controller, aphy_vector3* direction);
typedef aphy_error (*aphySetCharacterControllerVelocityForTimeInterval_FUN) (aphy_character_controller* character_controller, aphy_vector3 velocity, aphy_scalar time_interval);
typedef aphy_error (*aphySetCharacterControllerVelocityForTimeIntervalFrom_FUN) (aphy_character_controller* character_controller, aphy_vector3* velocity, aphy_scalar time_interval);
typedef aphy_error (*aphyWarpCharacterController_FUN) (aphy_character_controller* character_controller, aphy_vector3 origin);
typedef aphy_error (*aphyWarpCharacterControllerWithOriginFrom_FUN) (aphy_character_controller* character_controller, aphy_vector3* origin);
typedef aphy_bool (*aphyCanCharacterControllerJump_FUN) (aphy_character_controller* character_controller);
typedef aphy_error (*aphyCharacterControllerJump_FUN) (aphy_character_controller* character_controller);
typedef aphy_bool (*aphyIsCharacterControllerOnGround_FUN) (aphy_character_controller* character_controller);
typedef aphy_error (*aphySetCharacterMaxJumpHeight_FUN) (aphy_character_controller* character_controller, aphy_scalar height);
typedef aphy_error (*aphySetCharacterJumpSpeed_FUN) (aphy_character_controller* character_controller, aphy_scalar speed);
typedef aphy_error (*aphySetCharacterGravity_FUN) (aphy_character_controller* character_controller, aphy_scalar gravity);

APHY_EXPORT aphy_error aphyAddCharacterControllerReference(aphy_character_controller* character_controller);
APHY_EXPORT aphy_error aphyReleaseCharacterControllerReference(aphy_character_controller* character_controller);
APHY_EXPORT aphy_error aphySetCharacterControllerWalkDirection(aphy_character_controller* character_controller, aphy_vector3 direction);
APHY_EXPORT aphy_error aphySetCharacterControllerWalkDirectionFrom(aphy_character_controller* character_controller, aphy_vector3* direction);
APHY_EXPORT aphy_error aphySetCharacterControllerVelocityForTimeInterval(aphy_character_controller* character_controller, aphy_vector3 velocity, aphy_scalar time_interval);
APHY_EXPORT aphy_error aphySetCharacterControllerVelocityForTimeIntervalFrom(aphy_character_controller* character_controller, aphy_vector3* velocity, aphy_scalar time_interval);
APHY_EXPORT aphy_error aphyWarpCharacterController(aphy_character_controller* character_controller, aphy_vector3 origin);
APHY_EXPORT aphy_error aphyWarpCharacterControllerWithOriginFrom(aphy_character_controller* character_controller, aphy_vector3* origin);
APHY_EXPORT aphy_bool aphyCanCharacterControllerJump(aphy_character_controller* character_controller);
APHY_EXPORT aphy_error aphyCharacterControllerJump(aphy_character_controller* character_controller);
APHY_EXPORT aphy_bool aphyIsCharacterControllerOnGround(aphy_character_controller* character_controller);
APHY_EXPORT aphy_error aphySetCharacterMaxJumpHeight(aphy_character_controller* character_controller, aphy_scalar height);
APHY_EXPORT aphy_error aphySetCharacterJumpSpeed(aphy_character_controller* character_controller, aphy_scalar speed);
APHY_EXPORT aphy_error aphySetCharacterGravity(aphy_character_controller* character_controller, aphy_scalar gravity);

/* Methods for interface aphy_collision_object. */
typedef aphy_error (*aphyAddCollisionObjectReference_FUN) (aphy_collision_object* collision_object);
typedef aphy_error (*aphyReleaseCollisionObjectReference_FUN) (aphy_collision_object* collision_object);
typedef aphy_transform (*aphyGetCollisionObjectTransform_FUN) (aphy_collision_object* collision_object);
typedef aphy_error (*aphyGetCollisionObjectTransformInto_FUN) (aphy_collision_object* collision_object, aphy_transform* result);
typedef aphy_vector3 (*aphyGetCollisionObjectTranslation_FUN) (aphy_collision_object* collision_object);
typedef aphy_error (*aphyGetCollisionObjectTranslationInto_FUN) (aphy_collision_object* collision_object, aphy_vector3* result);
typedef aphy_matrix3x3 (*aphyGetCollisionObjectMatrix_FUN) (aphy_collision_object* collision_object);
typedef aphy_error (*aphyGetCollisionObjectMatrixInto_FUN) (aphy_collision_object* collision_object, aphy_matrix3x3* result);
typedef aphy_quaternion (*aphyGetCollisionObjectQuaternion_FUN) (aphy_collision_object* collision_object);
typedef aphy_error (*aphyGetCollisionObjectQuaternionInto_FUN) (aphy_collision_object* collision_object, aphy_quaternion* result);
typedef aphy_error (*aphySetCollisionObjectTransform_FUN) (aphy_collision_object* collision_object, aphy_transform value);
typedef aphy_error (*aphySetCollisionObjectTransformFrom_FUN) (aphy_collision_object* collision_object, aphy_transform* value);
typedef aphy_error (*aphySetCollisionObjectTranslation_FUN) (aphy_collision_object* collision_object, aphy_vector3 value);
typedef aphy_error (*aphySetCollisionObjectTranslationFrom_FUN) (aphy_collision_object* collision_object, aphy_vector3* value);
typedef aphy_error (*aphySetCollisionObjectMatrix_FUN) (aphy_collision_object* collision_object, aphy_matrix3x3 value);
typedef aphy_error (*aphySetCollisionObjectMatrixFrom_FUN) (aphy_collision_object* collision_object, aphy_matrix3x3* value);
typedef aphy_error (*aphySetCollisionObjectQuaternion_FUN) (aphy_collision_object* collision_object, aphy_quaternion value);
typedef aphy_error (*aphySetCollisionObjectQuaternionFrom_FUN) (aphy_collision_object* collision_object, aphy_quaternion* value);
typedef aphy_error (*aphySetCollisionObjectShape_FUN) (aphy_collision_object* collision_object, aphy_collision_shape* shape);
typedef aphy_error (*aphySetCollisionObjectHasContactResponse_FUN) (aphy_collision_object* collision_object, aphy_bool value);
typedef aphy_error (*aphySetCollisionObjectIsStatic_FUN) (aphy_collision_object* collision_object, aphy_bool value);
typedef aphy_error (*aphySetCollisionObjectIsKinematicObject_FUN) (aphy_collision_object* collision_object, aphy_bool value);
typedef aphy_error (*aphySetCollisionObjectIsCharacterObject_FUN) (aphy_collision_object* collision_object, aphy_bool value);
typedef aphy_error (*aphySetCollisionObjectDebugDrawingEnabled_FUN) (aphy_collision_object* collision_object, aphy_bool value);
typedef aphy_size (*aphyGetGhostCollisionObjectOverlappingObjectCount_FUN) (aphy_collision_object* collision_object);
typedef aphy_collision_object* (*aphyGetGhostCollisionObjectOverlappingObject_FUN) (aphy_collision_object* collision_object, aphy_size index);
typedef aphy_error (*aphyActivateRigidBody_FUN) (aphy_collision_object* collision_object);
typedef aphy_error (*aphySetRigidBodyLinearVelocityFrom_FUN) (aphy_collision_object* collision_object, aphy_vector3* velocity);
typedef aphy_error (*aphySetRigidBodyAngularVelocityFrom_FUN) (aphy_collision_object* collision_object, aphy_vector3* velocity);

APHY_EXPORT aphy_error aphyAddCollisionObjectReference(aphy_collision_object* collision_object);
APHY_EXPORT aphy_error aphyReleaseCollisionObjectReference(aphy_collision_object* collision_object);
APHY_EXPORT aphy_transform aphyGetCollisionObjectTransform(aphy_collision_object* collision_object);
APHY_EXPORT aphy_error aphyGetCollisionObjectTransformInto(aphy_collision_object* collision_object, aphy_transform* result);
APHY_EXPORT aphy_vector3 aphyGetCollisionObjectTranslation(aphy_collision_object* collision_object);
APHY_EXPORT aphy_error aphyGetCollisionObjectTranslationInto(aphy_collision_object* collision_object, aphy_vector3* result);
APHY_EXPORT aphy_matrix3x3 aphyGetCollisionObjectMatrix(aphy_collision_object* collision_object);
APHY_EXPORT aphy_error aphyGetCollisionObjectMatrixInto(aphy_collision_object* collision_object, aphy_matrix3x3* result);
APHY_EXPORT aphy_quaternion aphyGetCollisionObjectQuaternion(aphy_collision_object* collision_object);
APHY_EXPORT aphy_error aphyGetCollisionObjectQuaternionInto(aphy_collision_object* collision_object, aphy_quaternion* result);
APHY_EXPORT aphy_error aphySetCollisionObjectTransform(aphy_collision_object* collision_object, aphy_transform value);
APHY_EXPORT aphy_error aphySetCollisionObjectTransformFrom(aphy_collision_object* collision_object, aphy_transform* value);
APHY_EXPORT aphy_error aphySetCollisionObjectTranslation(aphy_collision_object* collision_object, aphy_vector3 value);
APHY_EXPORT aphy_error aphySetCollisionObjectTranslationFrom(aphy_collision_object* collision_object, aphy_vector3* value);
APHY_EXPORT aphy_error aphySetCollisionObjectMatrix(aphy_collision_object* collision_object, aphy_matrix3x3 value);
APHY_EXPORT aphy_error aphySetCollisionObjectMatrixFrom(aphy_collision_object* collision_object, aphy_matrix3x3* value);
APHY_EXPORT aphy_error aphySetCollisionObjectQuaternion(aphy_collision_object* collision_object, aphy_quaternion value);
APHY_EXPORT aphy_error aphySetCollisionObjectQuaternionFrom(aphy_collision_object* collision_object, aphy_quaternion* value);
APHY_EXPORT aphy_error aphySetCollisionObjectShape(aphy_collision_object* collision_object, aphy_collision_shape* shape);
APHY_EXPORT aphy_error aphySetCollisionObjectHasContactResponse(aphy_collision_object* collision_object, aphy_bool value);
APHY_EXPORT aphy_error aphySetCollisionObjectIsStatic(aphy_collision_object* collision_object, aphy_bool value);
APHY_EXPORT aphy_error aphySetCollisionObjectIsKinematicObject(aphy_collision_object* collision_object, aphy_bool value);
APHY_EXPORT aphy_error aphySetCollisionObjectIsCharacterObject(aphy_collision_object* collision_object, aphy_bool value);
APHY_EXPORT aphy_error aphySetCollisionObjectDebugDrawingEnabled(aphy_collision_object* collision_object, aphy_bool value);
APHY_EXPORT aphy_size aphyGetGhostCollisionObjectOverlappingObjectCount(aphy_collision_object* collision_object);
APHY_EXPORT aphy_collision_object* aphyGetGhostCollisionObjectOverlappingObject(aphy_collision_object* collision_object, aphy_size index);
APHY_EXPORT aphy_error aphyActivateRigidBody(aphy_collision_object* collision_object);
APHY_EXPORT aphy_error aphySetRigidBodyLinearVelocityFrom(aphy_collision_object* collision_object, aphy_vector3* velocity);
APHY_EXPORT aphy_error aphySetRigidBodyAngularVelocityFrom(aphy_collision_object* collision_object, aphy_vector3* velocity);

/* Methods for interface aphy_collision_shape. */
typedef aphy_error (*aphyAddCollisionShapeReference_FUN) (aphy_collision_shape* collision_shape);
typedef aphy_error (*aphyReleaseCollisionShapeReference_FUN) (aphy_collision_shape* collision_shape);
typedef aphy_error (*aphySetShapeMargin_FUN) (aphy_collision_shape* collision_shape, aphy_scalar margin);
typedef aphy_scalar (*aphyGetShapeMargin_FUN) (aphy_collision_shape* collision_shape);
typedef aphy_vector3 (*aphyComputeLocalInertia_FUN) (aphy_collision_shape* collision_shape, aphy_scalar mass);
typedef aphy_error (*aphyComputeLocalInertiaInto_FUN) (aphy_collision_shape* collision_shape, aphy_scalar mass, aphy_vector3* result);
typedef aphy_error (*aphyAddLocalShapeWithTransform_FUN) (aphy_collision_shape* collision_shape, aphy_collision_shape* shape, aphy_transform transform);
typedef aphy_error (*aphyAddLocalShapeWithTransformFrom_FUN) (aphy_collision_shape* collision_shape, aphy_collision_shape* shape, aphy_transform* transform);

APHY_EXPORT aphy_error aphyAddCollisionShapeReference(aphy_collision_shape* collision_shape);
APHY_EXPORT aphy_error aphyReleaseCollisionShapeReference(aphy_collision_shape* collision_shape);
APHY_EXPORT aphy_error aphySetShapeMargin(aphy_collision_shape* collision_shape, aphy_scalar margin);
APHY_EXPORT aphy_scalar aphyGetShapeMargin(aphy_collision_shape* collision_shape);
APHY_EXPORT aphy_vector3 aphyComputeLocalInertia(aphy_collision_shape* collision_shape, aphy_scalar mass);
APHY_EXPORT aphy_error aphyComputeLocalInertiaInto(aphy_collision_shape* collision_shape, aphy_scalar mass, aphy_vector3* result);
APHY_EXPORT aphy_error aphyAddLocalShapeWithTransform(aphy_collision_shape* collision_shape, aphy_collision_shape* shape, aphy_transform transform);
APHY_EXPORT aphy_error aphyAddLocalShapeWithTransformFrom(aphy_collision_shape* collision_shape, aphy_collision_shape* shape, aphy_transform* transform);

/* Methods for interface aphy_motion_state. */
typedef aphy_error (*aphyAddMotionStateReference_FUN) (aphy_motion_state* motion_state);
typedef aphy_error (*aphyReleaseMotionStateReference_FUN) (aphy_motion_state* motion_state);
typedef aphy_transform (*aphyGetMotionStateTransform_FUN) (aphy_motion_state* motion_state);
typedef aphy_error (*aphyGetMotionStateTransformInto_FUN) (aphy_motion_state* motion_state, aphy_transform* result);
typedef aphy_vector3 (*aphyGetMotionStateTranslation_FUN) (aphy_motion_state* motion_state);
typedef aphy_error (*aphyGetMotionStateTranslationInto_FUN) (aphy_motion_state* motion_state, aphy_vector3* result);
typedef aphy_matrix3x3 (*aphyGetMotionStateMatrix_FUN) (aphy_motion_state* motion_state);
typedef aphy_error (*aphyGetMotionStateMatrixInto_FUN) (aphy_motion_state* motion_state, aphy_matrix3x3* result);
typedef aphy_quaternion (*aphyGetMotionStateQuaternion_FUN) (aphy_motion_state* motion_state);
typedef aphy_error (*aphyGetMotionStateQuaternionInto_FUN) (aphy_motion_state* motion_state, aphy_quaternion* result);
typedef aphy_error (*aphySetMotionStateTransform_FUN) (aphy_motion_state* motion_state, aphy_transform value);
typedef aphy_error (*aphySetMotionStateTransformFrom_FUN) (aphy_motion_state* motion_state, aphy_transform* value);
typedef aphy_error (*aphySetMotionStateTranslation_FUN) (aphy_motion_state* motion_state, aphy_vector3 value);
typedef aphy_error (*aphySetMotionStateTranslationFrom_FUN) (aphy_motion_state* motion_state, aphy_vector3* value);
typedef aphy_error (*aphySetMotionStateMatrix_FUN) (aphy_motion_state* motion_state, aphy_matrix3x3 value);
typedef aphy_error (*aphySetMotionStateMatrixFrom_FUN) (aphy_motion_state* motion_state, aphy_matrix3x3* value);
typedef aphy_error (*aphySetMotionStateQuaternion_FUN) (aphy_motion_state* motion_state, aphy_quaternion value);
typedef aphy_error (*aphySetMotionStateQuaternionFrom_FUN) (aphy_motion_state* motion_state, aphy_quaternion* value);

APHY_EXPORT aphy_error aphyAddMotionStateReference(aphy_motion_state* motion_state);
APHY_EXPORT aphy_error aphyReleaseMotionStateReference(aphy_motion_state* motion_state);
APHY_EXPORT aphy_transform aphyGetMotionStateTransform(aphy_motion_state* motion_state);
APHY_EXPORT aphy_error aphyGetMotionStateTransformInto(aphy_motion_state* motion_state, aphy_transform* result);
APHY_EXPORT aphy_vector3 aphyGetMotionStateTranslation(aphy_motion_state* motion_state);
APHY_EXPORT aphy_error aphyGetMotionStateTranslationInto(aphy_motion_state* motion_state, aphy_vector3* result);
APHY_EXPORT aphy_matrix3x3 aphyGetMotionStateMatrix(aphy_motion_state* motion_state);
APHY_EXPORT aphy_error aphyGetMotionStateMatrixInto(aphy_motion_state* motion_state, aphy_matrix3x3* result);
APHY_EXPORT aphy_quaternion aphyGetMotionStateQuaternion(aphy_motion_state* motion_state);
APHY_EXPORT aphy_error aphyGetMotionStateQuaternionInto(aphy_motion_state* motion_state, aphy_quaternion* result);
APHY_EXPORT aphy_error aphySetMotionStateTransform(aphy_motion_state* motion_state, aphy_transform value);
APHY_EXPORT aphy_error aphySetMotionStateTransformFrom(aphy_motion_state* motion_state, aphy_transform* value);
APHY_EXPORT aphy_error aphySetMotionStateTranslation(aphy_motion_state* motion_state, aphy_vector3 value);
APHY_EXPORT aphy_error aphySetMotionStateTranslationFrom(aphy_motion_state* motion_state, aphy_vector3* value);
APHY_EXPORT aphy_error aphySetMotionStateMatrix(aphy_motion_state* motion_state, aphy_matrix3x3 value);
APHY_EXPORT aphy_error aphySetMotionStateMatrixFrom(aphy_motion_state* motion_state, aphy_matrix3x3* value);
APHY_EXPORT aphy_error aphySetMotionStateQuaternion(aphy_motion_state* motion_state, aphy_quaternion value);
APHY_EXPORT aphy_error aphySetMotionStateQuaternionFrom(aphy_motion_state* motion_state, aphy_quaternion* value);

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
	aphyCreateDefaultMotionStte_FUN aphyCreateDefaultMotionStte;
	aphyCreateDynamicsWorld_FUN aphyCreateDynamicsWorld;
	aphyCreateBoxShape_FUN aphyCreateBoxShape;
	aphyCreateCylinderX_FUN aphyCreateCylinderX;
	aphyCreateCylinderY_FUN aphyCreateCylinderY;
	aphyCreateCylinderZ_FUN aphyCreateCylinderZ;
	aphyCreateConvexHullShape_FUN aphyCreateConvexHullShape;
	aphyCreateCapsuleShapeX_FUN aphyCreateCapsuleShapeX;
	aphyCreateCapsuleShapeY_FUN aphyCreateCapsuleShapeY;
	aphyCreateCapsuleShapeZ_FUN aphyCreateCapsuleShapeZ;
	aphyCreateCompoundShape_FUN aphyCreateCompoundShape;
	aphyCreateConeX_FUN aphyCreateConeX;
	aphyCreateConeY_FUN aphyCreateConeY;
	aphyCreateConeZ_FUN aphyCreateConeZ;
	aphyCreateEmptyShape_FUN aphyCreateEmptyShape;
	aphyCreateHeightfieldTerrainShape_FUN aphyCreateHeightfieldTerrainShape;
	aphyCreateCollisionMeshCollection_FUN aphyCreateCollisionMeshCollection;
	aphyCreateTriangleMeshCollisionShape_FUN aphyCreateTriangleMeshCollisionShape;
	aphyCreateSphere_FUN aphyCreateSphere;
	aphyCreateSimpleRigidBody_FUN aphyCreateSimpleRigidBody;
	aphyCreateSimpleRigidBodyFrom_FUN aphyCreateSimpleRigidBodyFrom;
	aphyCreateGhostObject_FUN aphyCreateGhostObject;
	aphyCreatePairCachingGhostObject_FUN aphyCreatePairCachingGhostObject;
	aphyCreateKinematicCharacterController_FUN aphyCreateKinematicCharacterController;
	aphyAddCollisionConfigurationReference_FUN aphyAddCollisionConfigurationReference;
	aphyReleaseCollisionConfiguration_FUN aphyReleaseCollisionConfiguration;
	aphyAddCollisionDispatcherReference_FUN aphyAddCollisionDispatcherReference;
	aphyReleaseCollisionDispatcher_FUN aphyReleaseCollisionDispatcher;
	aphyAddBroadphaseReference_FUN aphyAddBroadphaseReference;
	aphyReleaseBroadphaseReference_FUN aphyReleaseBroadphaseReference;
	aphyAddCollisionMeshCollectionReference_FUN aphyAddCollisionMeshCollectionReference;
	aphyReleaseCollisionMeshCollectionReference_FUN aphyReleaseCollisionMeshCollectionReference;
	aphyAddCollisionMeshAccessorToCollection_FUN aphyAddCollisionMeshAccessorToCollection;
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
	aphyAddCharacterController_FUN aphyAddCharacterController;
	aphyRemoveCharacterController_FUN aphyRemoveCharacterController;
	aphyAddRigidBodyWithFilter_FUN aphyAddRigidBodyWithFilter;
	aphyStepSimulation_FUN aphyStepSimulation;
	aphySetGravity_FUN aphySetGravity;
	aphyEncodeDebugDrawing_FUN aphyEncodeDebugDrawing;
	aphyGetDebugDrawingData_FUN aphyGetDebugDrawingData;
	aphyAddCharacterControllerReference_FUN aphyAddCharacterControllerReference;
	aphyReleaseCharacterControllerReference_FUN aphyReleaseCharacterControllerReference;
	aphySetCharacterControllerWalkDirection_FUN aphySetCharacterControllerWalkDirection;
	aphySetCharacterControllerWalkDirectionFrom_FUN aphySetCharacterControllerWalkDirectionFrom;
	aphySetCharacterControllerVelocityForTimeInterval_FUN aphySetCharacterControllerVelocityForTimeInterval;
	aphySetCharacterControllerVelocityForTimeIntervalFrom_FUN aphySetCharacterControllerVelocityForTimeIntervalFrom;
	aphyWarpCharacterController_FUN aphyWarpCharacterController;
	aphyWarpCharacterControllerWithOriginFrom_FUN aphyWarpCharacterControllerWithOriginFrom;
	aphyCanCharacterControllerJump_FUN aphyCanCharacterControllerJump;
	aphyCharacterControllerJump_FUN aphyCharacterControllerJump;
	aphyIsCharacterControllerOnGround_FUN aphyIsCharacterControllerOnGround;
	aphySetCharacterMaxJumpHeight_FUN aphySetCharacterMaxJumpHeight;
	aphySetCharacterJumpSpeed_FUN aphySetCharacterJumpSpeed;
	aphySetCharacterGravity_FUN aphySetCharacterGravity;
	aphyAddCollisionObjectReference_FUN aphyAddCollisionObjectReference;
	aphyReleaseCollisionObjectReference_FUN aphyReleaseCollisionObjectReference;
	aphyGetCollisionObjectTransform_FUN aphyGetCollisionObjectTransform;
	aphyGetCollisionObjectTransformInto_FUN aphyGetCollisionObjectTransformInto;
	aphyGetCollisionObjectTranslation_FUN aphyGetCollisionObjectTranslation;
	aphyGetCollisionObjectTranslationInto_FUN aphyGetCollisionObjectTranslationInto;
	aphyGetCollisionObjectMatrix_FUN aphyGetCollisionObjectMatrix;
	aphyGetCollisionObjectMatrixInto_FUN aphyGetCollisionObjectMatrixInto;
	aphyGetCollisionObjectQuaternion_FUN aphyGetCollisionObjectQuaternion;
	aphyGetCollisionObjectQuaternionInto_FUN aphyGetCollisionObjectQuaternionInto;
	aphySetCollisionObjectTransform_FUN aphySetCollisionObjectTransform;
	aphySetCollisionObjectTransformFrom_FUN aphySetCollisionObjectTransformFrom;
	aphySetCollisionObjectTranslation_FUN aphySetCollisionObjectTranslation;
	aphySetCollisionObjectTranslationFrom_FUN aphySetCollisionObjectTranslationFrom;
	aphySetCollisionObjectMatrix_FUN aphySetCollisionObjectMatrix;
	aphySetCollisionObjectMatrixFrom_FUN aphySetCollisionObjectMatrixFrom;
	aphySetCollisionObjectQuaternion_FUN aphySetCollisionObjectQuaternion;
	aphySetCollisionObjectQuaternionFrom_FUN aphySetCollisionObjectQuaternionFrom;
	aphySetCollisionObjectShape_FUN aphySetCollisionObjectShape;
	aphySetCollisionObjectHasContactResponse_FUN aphySetCollisionObjectHasContactResponse;
	aphySetCollisionObjectIsStatic_FUN aphySetCollisionObjectIsStatic;
	aphySetCollisionObjectIsKinematicObject_FUN aphySetCollisionObjectIsKinematicObject;
	aphySetCollisionObjectIsCharacterObject_FUN aphySetCollisionObjectIsCharacterObject;
	aphySetCollisionObjectDebugDrawingEnabled_FUN aphySetCollisionObjectDebugDrawingEnabled;
	aphyGetGhostCollisionObjectOverlappingObjectCount_FUN aphyGetGhostCollisionObjectOverlappingObjectCount;
	aphyGetGhostCollisionObjectOverlappingObject_FUN aphyGetGhostCollisionObjectOverlappingObject;
	aphyActivateRigidBody_FUN aphyActivateRigidBody;
	aphySetRigidBodyLinearVelocityFrom_FUN aphySetRigidBodyLinearVelocityFrom;
	aphySetRigidBodyAngularVelocityFrom_FUN aphySetRigidBodyAngularVelocityFrom;
	aphyAddCollisionShapeReference_FUN aphyAddCollisionShapeReference;
	aphyReleaseCollisionShapeReference_FUN aphyReleaseCollisionShapeReference;
	aphySetShapeMargin_FUN aphySetShapeMargin;
	aphyGetShapeMargin_FUN aphyGetShapeMargin;
	aphyComputeLocalInertia_FUN aphyComputeLocalInertia;
	aphyComputeLocalInertiaInto_FUN aphyComputeLocalInertiaInto;
	aphyAddLocalShapeWithTransform_FUN aphyAddLocalShapeWithTransform;
	aphyAddLocalShapeWithTransformFrom_FUN aphyAddLocalShapeWithTransformFrom;
	aphyAddMotionStateReference_FUN aphyAddMotionStateReference;
	aphyReleaseMotionStateReference_FUN aphyReleaseMotionStateReference;
	aphyGetMotionStateTransform_FUN aphyGetMotionStateTransform;
	aphyGetMotionStateTransformInto_FUN aphyGetMotionStateTransformInto;
	aphyGetMotionStateTranslation_FUN aphyGetMotionStateTranslation;
	aphyGetMotionStateTranslationInto_FUN aphyGetMotionStateTranslationInto;
	aphyGetMotionStateMatrix_FUN aphyGetMotionStateMatrix;
	aphyGetMotionStateMatrixInto_FUN aphyGetMotionStateMatrixInto;
	aphyGetMotionStateQuaternion_FUN aphyGetMotionStateQuaternion;
	aphyGetMotionStateQuaternionInto_FUN aphyGetMotionStateQuaternionInto;
	aphySetMotionStateTransform_FUN aphySetMotionStateTransform;
	aphySetMotionStateTransformFrom_FUN aphySetMotionStateTransformFrom;
	aphySetMotionStateTranslation_FUN aphySetMotionStateTranslation;
	aphySetMotionStateTranslationFrom_FUN aphySetMotionStateTranslationFrom;
	aphySetMotionStateMatrix_FUN aphySetMotionStateMatrix;
	aphySetMotionStateMatrixFrom_FUN aphySetMotionStateMatrixFrom;
	aphySetMotionStateQuaternion_FUN aphySetMotionStateQuaternion;
	aphySetMotionStateQuaternionFrom_FUN aphySetMotionStateQuaternionFrom;
} aphy_icd_dispatch;


#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* _APHY_H_ */
