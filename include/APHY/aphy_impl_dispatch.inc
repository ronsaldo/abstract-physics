
#include "aphy_impl.hpp"

inline void* hideType(void *t)
{
	return t;
}

#define asRef(O, I) (*reinterpret_cast<aphy::ref<O> *> (hideType(&I)) )
#define asRefCounter(O, I) (reinterpret_cast<aphy::ref_counter<O> *> (I))

//==============================================================================
// engine C dispatching functions.
//==============================================================================

APHY_EXPORT aphy_error aphyAddEngineReference(aphy_engine* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::engine, self)->retain();
}

APHY_EXPORT aphy_error aphyReleaseEngine(aphy_engine* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::engine, self)->release();
}

APHY_EXPORT aphy_cstring aphyGetEngineName(aphy_engine* self)
{
	return asRef(aphy::engine, self)->getName();
}

APHY_EXPORT aphy_int aphyGetEngineVersion(aphy_engine* self)
{
	return asRef(aphy::engine, self)->getVersion();
}

APHY_EXPORT aphy_collision_configuration* aphyCreateDefaultCollisionConfiguration(aphy_engine* self)
{
	return reinterpret_cast<aphy_collision_configuration*> (asRef(aphy::engine, self)->createDefaultCollisionConfiguration());
}

APHY_EXPORT aphy_collision_dispatcher* aphyCreateDefaultCollisionDispatcher(aphy_engine* self, aphy_collision_configuration* collision_configuration)
{
	return reinterpret_cast<aphy_collision_dispatcher*> (asRef(aphy::engine, self)->createDefaultCollisionDispatcher(asRef(aphy::collision_configuration, collision_configuration)));
}

APHY_EXPORT aphy_broadphase* aphyCreateDefaultBroadphase(aphy_engine* self)
{
	return reinterpret_cast<aphy_broadphase*> (asRef(aphy::engine, self)->createDefaultBroadphase());
}

APHY_EXPORT aphy_constraint_solver* aphyCreateDefaultConstraintSolver(aphy_engine* self)
{
	return reinterpret_cast<aphy_constraint_solver*> (asRef(aphy::engine, self)->createDefaultConstraintSolver());
}

APHY_EXPORT aphy_motion_state* aphyCreateDefaultMotionStte(aphy_engine* self)
{
	return reinterpret_cast<aphy_motion_state*> (asRef(aphy::engine, self)->createDefaultMotionState());
}

APHY_EXPORT aphy_world* aphyCreateDynamicsWorld(aphy_engine* self, aphy_collision_dispatcher* collision_dispatcher, aphy_broadphase* broadphase, aphy_constraint_solver* constraint_solver, aphy_collision_configuration* collision_configuration)
{
	return reinterpret_cast<aphy_world*> (asRef(aphy::engine, self)->createDynamicsWorld(asRef(aphy::collision_dispatcher, collision_dispatcher), asRef(aphy::broadphase, broadphase), asRef(aphy::constraint_solver, constraint_solver), asRef(aphy::collision_configuration, collision_configuration)));
}

APHY_EXPORT aphy_collision_shape* aphyCreateBoxShape(aphy_engine* self, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createBoxShape(half_width, half_height, half_depth));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCylinderX(aphy_engine* self, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createCylinderX(half_width, half_height, half_depth));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCylinderY(aphy_engine* self, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createCylinderY(half_width, half_height, half_depth));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCylinderZ(aphy_engine* self, aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createCylinderZ(half_width, half_height, half_depth));
}

APHY_EXPORT aphy_collision_shape* aphyCreateConvexHullShape(aphy_engine* self, aphy_scalar* coordinates, aphy_size num_points, aphy_size stride)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createConvexHullShape(coordinates, num_points, stride));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeX(aphy_engine* self, aphy_scalar radius, aphy_scalar height)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createCapsuleShapeX(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeY(aphy_engine* self, aphy_scalar radius, aphy_scalar height)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createCapsuleShapeY(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCapsuleShapeZ(aphy_engine* self, aphy_scalar radius, aphy_scalar height)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createCapsuleShapeZ(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateCompoundShape(aphy_engine* self)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createCompoundShape());
}

APHY_EXPORT aphy_collision_shape* aphyCreateConeX(aphy_engine* self, aphy_scalar radius, aphy_scalar height)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createConeX(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateConeY(aphy_engine* self, aphy_scalar radius, aphy_scalar height)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createConeY(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateConeZ(aphy_engine* self, aphy_scalar radius, aphy_scalar height)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createConeZ(radius, height));
}

APHY_EXPORT aphy_collision_shape* aphyCreateEmptyShape(aphy_engine* self)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createEmptyShape());
}

APHY_EXPORT aphy_collision_shape* aphyCreateHeightfieldTerrainShape(aphy_engine* self, aphy_int height_stick_width, aphy_int height_stick_length, aphy_pointer heightfield_data, aphy_scalar height_scale, aphy_scalar min_height, aphy_scalar max_height, aphy_axis up_axis, aphy_scalar_type height_data_type, aphy_bool flip_quad_edges, aphy_scalar local_scale_x, aphy_scalar local_scale_y, aphy_scalar local_scale_z)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createHeightfieldTerrainShape(height_stick_width, height_stick_length, heightfield_data, height_scale, min_height, max_height, up_axis, height_data_type, flip_quad_edges, local_scale_x, local_scale_y, local_scale_z));
}

APHY_EXPORT aphy_collision_mesh_collection* aphyCreateCollisionMeshCollection(aphy_engine* self)
{
	return reinterpret_cast<aphy_collision_mesh_collection*> (asRef(aphy::engine, self)->createCollisionMeshCollection());
}

APHY_EXPORT aphy_collision_shape* aphyCreateTriangleMeshCollisionShape(aphy_engine* self, aphy_collision_mesh_collection* mesh_collection)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createTriangleMeshCollisionShape(asRef(aphy::collision_mesh_collection, mesh_collection)));
}

APHY_EXPORT aphy_collision_shape* aphyCreateSphere(aphy_engine* self, aphy_scalar radius)
{
	return reinterpret_cast<aphy_collision_shape*> (asRef(aphy::engine, self)->createSphere(radius));
}

APHY_EXPORT aphy_collision_object* aphyCreateSimpleRigidBody(aphy_engine* self, aphy_scalar mass, aphy_motion_state* motion_state, aphy_collision_shape* collision_shape, aphy_vector3 local_inertia)
{
	return reinterpret_cast<aphy_collision_object*> (asRef(aphy::engine, self)->createSimpleRigidBody(mass, asRef(aphy::motion_state, motion_state), asRef(aphy::collision_shape, collision_shape), local_inertia));
}

APHY_EXPORT aphy_collision_object* aphyCreateSimpleRigidBodyFrom(aphy_engine* self, aphy_scalar mass, aphy_motion_state* motion_state, aphy_collision_shape* collision_shape, aphy_vector3* local_inertia)
{
	return reinterpret_cast<aphy_collision_object*> (asRef(aphy::engine, self)->createSimpleRigidBodyFrom(mass, asRef(aphy::motion_state, motion_state), asRef(aphy::collision_shape, collision_shape), local_inertia));
}

APHY_EXPORT aphy_collision_object* aphyCreateGhostObject(aphy_engine* self)
{
	return reinterpret_cast<aphy_collision_object*> (asRef(aphy::engine, self)->createGhostObject());
}

APHY_EXPORT aphy_collision_object* aphyCreatePairCachingGhostObject(aphy_engine* self)
{
	return reinterpret_cast<aphy_collision_object*> (asRef(aphy::engine, self)->createPairCachingGhostObject());
}

APHY_EXPORT aphy_character_controller* aphyCreateKinematicCharacterController(aphy_engine* self, aphy_collision_object* ghost_object, aphy_collision_shape* convex_shape, aphy_scalar step_height, aphy_axis up_axis)
{
	return reinterpret_cast<aphy_character_controller*> (asRef(aphy::engine, self)->createKinematicCharacterController(asRef(aphy::collision_object, ghost_object), asRef(aphy::collision_shape, convex_shape), step_height, up_axis));
}

//==============================================================================
// collision_configuration C dispatching functions.
//==============================================================================

APHY_EXPORT aphy_error aphyAddCollisionConfigurationReference(aphy_collision_configuration* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::collision_configuration, self)->retain();
}

APHY_EXPORT aphy_error aphyReleaseCollisionConfiguration(aphy_collision_configuration* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::collision_configuration, self)->release();
}

//==============================================================================
// collision_dispatcher C dispatching functions.
//==============================================================================

APHY_EXPORT aphy_error aphyAddCollisionDispatcherReference(aphy_collision_dispatcher* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::collision_dispatcher, self)->retain();
}

APHY_EXPORT aphy_error aphyReleaseCollisionDispatcher(aphy_collision_dispatcher* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::collision_dispatcher, self)->release();
}

//==============================================================================
// broadphase C dispatching functions.
//==============================================================================

APHY_EXPORT aphy_error aphyAddBroadphaseReference(aphy_broadphase* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::broadphase, self)->retain();
}

APHY_EXPORT aphy_error aphyReleaseBroadphaseReference(aphy_broadphase* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::broadphase, self)->release();
}

//==============================================================================
// collision_mesh_collection C dispatching functions.
//==============================================================================

APHY_EXPORT aphy_error aphyAddCollisionMeshCollectionReference(aphy_collision_mesh_collection* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::collision_mesh_collection, self)->retain();
}

APHY_EXPORT aphy_error aphyReleaseCollisionMeshCollectionReference(aphy_collision_mesh_collection* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::collision_mesh_collection, self)->release();
}

APHY_EXPORT aphy_error aphyAddCollisionMeshAccessorToCollection(aphy_collision_mesh_collection* self, aphy_collision_mesh_accessor* accessor, aphy_transform* transform)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_mesh_collection, self)->addCollisionMeshAccessor(accessor, transform);
}

//==============================================================================
// constraint_solver C dispatching functions.
//==============================================================================

APHY_EXPORT aphy_error aphyAddConstraintSolverReference(aphy_constraint_solver* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::constraint_solver, self)->retain();
}

APHY_EXPORT aphy_error aphyReleaseConstraintSolverReference(aphy_constraint_solver* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::constraint_solver, self)->release();
}

//==============================================================================
// world C dispatching functions.
//==============================================================================

APHY_EXPORT aphy_error aphyAddWorldReference(aphy_world* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::world, self)->retain();
}

APHY_EXPORT aphy_error aphyReleaseWorldReference(aphy_world* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::world, self)->release();
}

APHY_EXPORT aphy_uint aphyGetNumberOfCollisionObject(aphy_world* self)
{
	return asRef(aphy::world, self)->getNumberOfCollisionObject();
}

APHY_EXPORT aphy_uint aphyGetNumberOfConstraints(aphy_world* self)
{
	return asRef(aphy::world, self)->getNumberOfConstraints();
}

APHY_EXPORT aphy_error aphyAddCollisionObject(aphy_world* self, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::world, self)->addCollisionObject(asRef(aphy::collision_object, object), collision_filter_group, collision_filter_mask);
}

APHY_EXPORT aphy_error aphyRemoveCollisionObject(aphy_world* self, aphy_collision_object* object)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::world, self)->removeCollisionObject(asRef(aphy::collision_object, object));
}

APHY_EXPORT aphy_error aphyAddRigidBody(aphy_world* self, aphy_collision_object* object)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::world, self)->addRigidBody(asRef(aphy::collision_object, object));
}

APHY_EXPORT aphy_error aphyRemoveRigidBody(aphy_world* self, aphy_collision_object* object)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::world, self)->removeRigidBody(asRef(aphy::collision_object, object));
}

APHY_EXPORT aphy_error aphyAddCharacterController(aphy_world* self, aphy_character_controller* character)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::world, self)->addCharacterController(asRef(aphy::character_controller, character));
}

APHY_EXPORT aphy_error aphyRemoveCharacterController(aphy_world* self, aphy_character_controller* character)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::world, self)->removeCharacterController(asRef(aphy::character_controller, character));
}

APHY_EXPORT aphy_error aphyAddRigidBodyWithFilter(aphy_world* self, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::world, self)->addRigidBodyWithFilter(asRef(aphy::collision_object, object), collision_filter_group, collision_filter_mask);
}

APHY_EXPORT aphy_error aphyStepSimulation(aphy_world* self, aphy_scalar time_step, aphy_int max_sub_steps, aphy_scalar fixed_time_step)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::world, self)->stepSimulation(time_step, max_sub_steps, fixed_time_step);
}

APHY_EXPORT aphy_error aphySetGravity(aphy_world* self, aphy_scalar x, aphy_scalar y, aphy_scalar z)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::world, self)->setGravity(x, y, z);
}

APHY_EXPORT aphy_size aphyEncodeDebugDrawing(aphy_world* self)
{
	return asRef(aphy::world, self)->encodeDebugDrawing();
}

APHY_EXPORT aphy_error aphyGetDebugDrawingData(aphy_world* self, aphy_size buffer_size, aphy_pointer buffer)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::world, self)->getDebugDrawingData(buffer_size, buffer);
}

//==============================================================================
// character_controller C dispatching functions.
//==============================================================================

APHY_EXPORT aphy_error aphyAddCharacterControllerReference(aphy_character_controller* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::character_controller, self)->retain();
}

APHY_EXPORT aphy_error aphyReleaseCharacterControllerReference(aphy_character_controller* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::character_controller, self)->release();
}

APHY_EXPORT aphy_error aphySetCharacterControllerWalkDirection(aphy_character_controller* self, aphy_vector3 direction)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::character_controller, self)->setWalkDirection(direction);
}

APHY_EXPORT aphy_error aphySetCharacterControllerWalkDirectionFrom(aphy_character_controller* self, aphy_vector3* direction)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::character_controller, self)->setWalkDirectionFrom(direction);
}

APHY_EXPORT aphy_error aphySetCharacterControllerVelocityForTimeInterval(aphy_character_controller* self, aphy_vector3 velocity, aphy_scalar time_interval)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::character_controller, self)->setVelocityForTimeInterval(velocity, time_interval);
}

APHY_EXPORT aphy_error aphySetCharacterControllerVelocityForTimeIntervalFrom(aphy_character_controller* self, aphy_vector3* velocity, aphy_scalar time_interval)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::character_controller, self)->setVelocityForTimeIntervalFrom(velocity, time_interval);
}

APHY_EXPORT aphy_error aphyWarpCharacterController(aphy_character_controller* self, aphy_vector3 origin)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::character_controller, self)->warp(origin);
}

APHY_EXPORT aphy_error aphyWarpCharacterControllerWithOriginFrom(aphy_character_controller* self, aphy_vector3* origin)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::character_controller, self)->warpWithOriginFrom(origin);
}

APHY_EXPORT aphy_bool aphyCanCharacterControllerJump(aphy_character_controller* self)
{
	return asRef(aphy::character_controller, self)->canJump();
}

APHY_EXPORT aphy_error aphyCharacterControllerJump(aphy_character_controller* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::character_controller, self)->jump();
}

APHY_EXPORT aphy_bool aphyIsCharacterControllerOnGround(aphy_character_controller* self)
{
	return asRef(aphy::character_controller, self)->isOnGround();
}

APHY_EXPORT aphy_error aphySetCharacterMaxJumpHeight(aphy_character_controller* self, aphy_scalar height)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::character_controller, self)->setMaxJumpHeight(height);
}

APHY_EXPORT aphy_error aphySetCharacterJumpSpeed(aphy_character_controller* self, aphy_scalar speed)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::character_controller, self)->setJumpSpeed(speed);
}

APHY_EXPORT aphy_error aphySetCharacterGravity(aphy_character_controller* self, aphy_scalar gravity)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::character_controller, self)->setGravity(gravity);
}

//==============================================================================
// collision_object C dispatching functions.
//==============================================================================

APHY_EXPORT aphy_error aphyAddCollisionObjectReference(aphy_collision_object* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::collision_object, self)->retain();
}

APHY_EXPORT aphy_error aphyReleaseCollisionObjectReference(aphy_collision_object* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::collision_object, self)->release();
}

APHY_EXPORT aphy_transform aphyGetCollisionObjectTransform(aphy_collision_object* self)
{
	return asRef(aphy::collision_object, self)->getTransform();
}

APHY_EXPORT aphy_error aphyGetCollisionObjectTransformInto(aphy_collision_object* self, aphy_transform* result)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->getTransformInto(result);
}

APHY_EXPORT aphy_vector3 aphyGetCollisionObjectTranslation(aphy_collision_object* self)
{
	return asRef(aphy::collision_object, self)->getTranslation();
}

APHY_EXPORT aphy_error aphyGetCollisionObjectTranslationInto(aphy_collision_object* self, aphy_vector3* result)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->getTranslationInto(result);
}

APHY_EXPORT aphy_matrix3x3 aphyGetCollisionObjectMatrix(aphy_collision_object* self)
{
	return asRef(aphy::collision_object, self)->getMatrix();
}

APHY_EXPORT aphy_error aphyGetCollisionObjectMatrixInto(aphy_collision_object* self, aphy_matrix3x3* result)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->getMatrixInto(result);
}

APHY_EXPORT aphy_quaternion aphyGetCollisionObjectQuaternion(aphy_collision_object* self)
{
	return asRef(aphy::collision_object, self)->getQuaternion();
}

APHY_EXPORT aphy_error aphyGetCollisionObjectQuaternionInto(aphy_collision_object* self, aphy_quaternion* result)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->getQuaternionInto(result);
}

APHY_EXPORT aphy_error aphySetCollisionObjectTransform(aphy_collision_object* self, aphy_transform value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setTransform(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectTransformFrom(aphy_collision_object* self, aphy_transform* value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setTransformFrom(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectTranslation(aphy_collision_object* self, aphy_vector3 value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setTranslation(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectTranslationFrom(aphy_collision_object* self, aphy_vector3* value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setTranslationFrom(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectMatrix(aphy_collision_object* self, aphy_matrix3x3 value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setMatrix(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectMatrixFrom(aphy_collision_object* self, aphy_matrix3x3* value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setMatrixFrom(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectQuaternion(aphy_collision_object* self, aphy_quaternion value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setQuaternion(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectQuaternionFrom(aphy_collision_object* self, aphy_quaternion* value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setQuaternion(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectShape(aphy_collision_object* self, aphy_collision_shape* shape)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setCollisionShape(asRef(aphy::collision_shape, shape));
}

APHY_EXPORT aphy_error aphySetCollisionObjectHasContactResponse(aphy_collision_object* self, aphy_bool value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setHasContactResponse(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectIsStatic(aphy_collision_object* self, aphy_bool value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setIsStaticObject(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectIsKinematicObject(aphy_collision_object* self, aphy_bool value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setIsKinematicObject(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectIsCharacterObject(aphy_collision_object* self, aphy_bool value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setIsCharacterObject(value);
}

APHY_EXPORT aphy_error aphySetCollisionObjectDebugDrawingEnabled(aphy_collision_object* self, aphy_bool value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setDebugDrawingEnabled(value);
}

APHY_EXPORT aphy_size aphyGetGhostCollisionObjectOverlappingObjectCount(aphy_collision_object* self)
{
	return asRef(aphy::collision_object, self)->getOverlappingObjectCount();
}

APHY_EXPORT aphy_collision_object* aphyGetGhostCollisionObjectOverlappingObject(aphy_collision_object* self, aphy_size index)
{
	return reinterpret_cast<aphy_collision_object*> (asRef(aphy::collision_object, self)->getOverlappingObject(index));
}

APHY_EXPORT aphy_error aphyActivateRigidBody(aphy_collision_object* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->activate();
}

APHY_EXPORT aphy_error aphySetRigidBodyLinearVelocityFrom(aphy_collision_object* self, aphy_vector3* velocity)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setLinearVelocityFrom(velocity);
}

APHY_EXPORT aphy_error aphySetRigidBodyAngularVelocityFrom(aphy_collision_object* self, aphy_vector3* velocity)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_object, self)->setAngularVelocityFrom(velocity);
}

//==============================================================================
// collision_shape C dispatching functions.
//==============================================================================

APHY_EXPORT aphy_error aphyAddCollisionShapeReference(aphy_collision_shape* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::collision_shape, self)->retain();
}

APHY_EXPORT aphy_error aphyReleaseCollisionShapeReference(aphy_collision_shape* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::collision_shape, self)->release();
}

APHY_EXPORT aphy_error aphySetShapeMargin(aphy_collision_shape* self, aphy_scalar margin)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_shape, self)->setMargin(margin);
}

APHY_EXPORT aphy_scalar aphyGetShapeMargin(aphy_collision_shape* self)
{
	return asRef(aphy::collision_shape, self)->getMargin();
}

APHY_EXPORT aphy_vector3 aphyComputeLocalInertia(aphy_collision_shape* self, aphy_scalar mass)
{
	return asRef(aphy::collision_shape, self)->computeLocalInertia(mass);
}

APHY_EXPORT aphy_error aphyComputeLocalInertiaInto(aphy_collision_shape* self, aphy_scalar mass, aphy_vector3* result)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_shape, self)->computeLocalInertiaInto(mass, result);
}

APHY_EXPORT aphy_error aphyAddLocalShapeWithTransform(aphy_collision_shape* self, aphy_collision_shape* shape, aphy_transform transform)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_shape, self)->addLocalShapeWithTransform(asRef(aphy::collision_shape, shape), transform);
}

APHY_EXPORT aphy_error aphyAddLocalShapeWithTransformFrom(aphy_collision_shape* self, aphy_collision_shape* shape, aphy_transform* transform)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::collision_shape, self)->addLocalShapeWithTransformFrom(asRef(aphy::collision_shape, shape), transform);
}

//==============================================================================
// motion_state C dispatching functions.
//==============================================================================

APHY_EXPORT aphy_error aphyAddMotionStateReference(aphy_motion_state* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::motion_state, self)->retain();
}

APHY_EXPORT aphy_error aphyReleaseMotionStateReference(aphy_motion_state* self)
{
	if(!self) return APHY_NULL_POINTER;
	return asRefCounter(aphy::motion_state, self)->release();
}

APHY_EXPORT aphy_transform aphyGetMotionStateTransform(aphy_motion_state* self)
{
	return asRef(aphy::motion_state, self)->getTransform();
}

APHY_EXPORT aphy_error aphyGetMotionStateTransformInto(aphy_motion_state* self, aphy_transform* result)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->getTransformInto(result);
}

APHY_EXPORT aphy_vector3 aphyGetMotionStateTranslation(aphy_motion_state* self)
{
	return asRef(aphy::motion_state, self)->getTranslation();
}

APHY_EXPORT aphy_error aphyGetMotionStateTranslationInto(aphy_motion_state* self, aphy_vector3* result)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->getTranslationInto(result);
}

APHY_EXPORT aphy_matrix3x3 aphyGetMotionStateMatrix(aphy_motion_state* self)
{
	return asRef(aphy::motion_state, self)->getMatrix();
}

APHY_EXPORT aphy_error aphyGetMotionStateMatrixInto(aphy_motion_state* self, aphy_matrix3x3* result)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->getMatrixInto(result);
}

APHY_EXPORT aphy_quaternion aphyGetMotionStateQuaternion(aphy_motion_state* self)
{
	return asRef(aphy::motion_state, self)->getQuaternion();
}

APHY_EXPORT aphy_error aphyGetMotionStateQuaternionInto(aphy_motion_state* self, aphy_quaternion* result)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->getQuaternionInto(result);
}

APHY_EXPORT aphy_error aphySetMotionStateTransform(aphy_motion_state* self, aphy_transform value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->setTransform(value);
}

APHY_EXPORT aphy_error aphySetMotionStateTransformFrom(aphy_motion_state* self, aphy_transform* value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->setTransformFrom(value);
}

APHY_EXPORT aphy_error aphySetMotionStateTranslation(aphy_motion_state* self, aphy_vector3 value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->setTranslation(value);
}

APHY_EXPORT aphy_error aphySetMotionStateTranslationFrom(aphy_motion_state* self, aphy_vector3* value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->setTranslationFrom(value);
}

APHY_EXPORT aphy_error aphySetMotionStateMatrix(aphy_motion_state* self, aphy_matrix3x3 value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->setMatrix(value);
}

APHY_EXPORT aphy_error aphySetMotionStateMatrixFrom(aphy_motion_state* self, aphy_matrix3x3* value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->setMatrixFrom(value);
}

APHY_EXPORT aphy_error aphySetMotionStateQuaternion(aphy_motion_state* self, aphy_quaternion value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->setQuaternion(value);
}

APHY_EXPORT aphy_error aphySetMotionStateQuaternionFrom(aphy_motion_state* self, aphy_quaternion* value)
{
	if(!self) return APHY_NULL_POINTER;
	return asRef(aphy::motion_state, self)->setQuaternionFrom(value);
}



#undef asRef
#undef asRefCounter

namespace aphy
{
aphy_icd_dispatch cppRefcountedDispatchTable = {
#include "aphy_icd.10.inc"
};
} // End of aphy
