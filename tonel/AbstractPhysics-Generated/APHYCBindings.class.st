Class {
	#name : #APHYCBindings,
	#pools : [
		'APHYConstants',
		'APHYTypes'
	],
	#superclass : #APHYCBindingsBase,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'engine' }
APHYCBindings >> addReference_engine: engine [
	^ self ffiCall: #(aphy_error aphyAddEngineReference (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> release_engine: engine [
	^ self ffiCall: #(aphy_error aphyReleaseEngine (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> getName_engine: engine [
	^ self ffiCall: #(aphy_cstring aphyGetEngineName (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> getVersion_engine: engine [
	^ self ffiCall: #(aphy_int aphyGetEngineVersion (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> createDefaultCollisionConfiguration_engine: engine [
	^ self ffiCall: #(aphy_collision_configuration* aphyCreateDefaultCollisionConfiguration (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> createDefaultCollisionDispatcher_engine: engine collision_configuration: collision_configuration [
	^ self ffiCall: #(aphy_collision_dispatcher* aphyCreateDefaultCollisionDispatcher (aphy_engine* engine , aphy_collision_configuration* collision_configuration) )
]

{ #category : #'engine' }
APHYCBindings >> createDefaultBroadphase_engine: engine [
	^ self ffiCall: #(aphy_broadphase* aphyCreateDefaultBroadphase (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> createDefaultConstraintSolver_engine: engine [
	^ self ffiCall: #(aphy_constraint_solver* aphyCreateDefaultConstraintSolver (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> createDefaultMotionState_engine: engine [
	^ self ffiCall: #(aphy_motion_state* aphyCreateDefaultMotionStte (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> createDynamicsWorld_engine: engine collision_dispatcher: collision_dispatcher broadphase: broadphase constraint_solver: constraint_solver collision_configuration: collision_configuration [
	^ self ffiCall: #(aphy_world* aphyCreateDynamicsWorld (aphy_engine* engine , aphy_collision_dispatcher* collision_dispatcher , aphy_broadphase* broadphase , aphy_constraint_solver* constraint_solver , aphy_collision_configuration* collision_configuration) )
]

{ #category : #'engine' }
APHYCBindings >> createBoxShape_engine: engine half_width: half_width half_height: half_height half_depth: half_depth [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateBoxShape (aphy_engine* engine , aphy_scalar half_width , aphy_scalar half_height , aphy_scalar half_depth) )
]

{ #category : #'engine' }
APHYCBindings >> createCylinderX_engine: engine half_width: half_width half_height: half_height half_depth: half_depth [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateCylinderX (aphy_engine* engine , aphy_scalar half_width , aphy_scalar half_height , aphy_scalar half_depth) )
]

{ #category : #'engine' }
APHYCBindings >> createCylinderY_engine: engine half_width: half_width half_height: half_height half_depth: half_depth [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateCylinderY (aphy_engine* engine , aphy_scalar half_width , aphy_scalar half_height , aphy_scalar half_depth) )
]

{ #category : #'engine' }
APHYCBindings >> createCylinderZ_engine: engine half_width: half_width half_height: half_height half_depth: half_depth [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateCylinderZ (aphy_engine* engine , aphy_scalar half_width , aphy_scalar half_height , aphy_scalar half_depth) )
]

{ #category : #'engine' }
APHYCBindings >> createConvexHullShape_engine: engine coordinates: coordinates num_points: num_points stride: stride [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateConvexHullShape (aphy_engine* engine , aphy_scalar* coordinates , aphy_size num_points , aphy_size stride) )
]

{ #category : #'engine' }
APHYCBindings >> createCapsuleShapeX_engine: engine radius: radius height: height [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateCapsuleShapeX (aphy_engine* engine , aphy_scalar radius , aphy_scalar height) )
]

{ #category : #'engine' }
APHYCBindings >> createCapsuleShapeY_engine: engine radius: radius height: height [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateCapsuleShapeY (aphy_engine* engine , aphy_scalar radius , aphy_scalar height) )
]

{ #category : #'engine' }
APHYCBindings >> createCapsuleShapeZ_engine: engine radius: radius height: height [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateCapsuleShapeZ (aphy_engine* engine , aphy_scalar radius , aphy_scalar height) )
]

{ #category : #'engine' }
APHYCBindings >> createCompoundShape_engine: engine [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateCompoundShape (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> createConeX_engine: engine radius: radius height: height [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateConeX (aphy_engine* engine , aphy_scalar radius , aphy_scalar height) )
]

{ #category : #'engine' }
APHYCBindings >> createConeY_engine: engine radius: radius height: height [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateConeY (aphy_engine* engine , aphy_scalar radius , aphy_scalar height) )
]

{ #category : #'engine' }
APHYCBindings >> createConeZ_engine: engine radius: radius height: height [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateConeZ (aphy_engine* engine , aphy_scalar radius , aphy_scalar height) )
]

{ #category : #'engine' }
APHYCBindings >> createEmptyShape_engine: engine [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateEmptyShape (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> createHeightfieldTerrainShape_engine: engine height_stick_width: height_stick_width height_stick_length: height_stick_length heightfield_data: heightfield_data height_scale: height_scale min_height: min_height max_height: max_height up_axis: up_axis height_data_type: height_data_type flip_quad_edges: flip_quad_edges local_scale_x: local_scale_x local_scale_y: local_scale_y local_scale_z: local_scale_z [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateHeightfieldTerrainShape (aphy_engine* engine , aphy_int height_stick_width , aphy_int height_stick_length , aphy_pointer heightfield_data , aphy_scalar height_scale , aphy_scalar min_height , aphy_scalar max_height , aphy_axis up_axis , aphy_scalar_type height_data_type , aphy_bool flip_quad_edges , aphy_scalar local_scale_x , aphy_scalar local_scale_y , aphy_scalar local_scale_z) )
]

{ #category : #'engine' }
APHYCBindings >> createSphere_engine: engine radius: radius [
	^ self ffiCall: #(aphy_collision_shape* aphyCreateSphere (aphy_engine* engine , aphy_scalar radius) )
]

{ #category : #'engine' }
APHYCBindings >> createSimpleRigidBody_engine: engine mass: mass motion_state: motion_state collision_shape: collision_shape local_inertia: local_inertia [
	^ self ffiCall: #(aphy_collision_object* aphyCreateSimpleRigidBody (aphy_engine* engine , aphy_scalar mass , aphy_motion_state* motion_state , aphy_collision_shape* collision_shape , aphy_vector3 local_inertia) )
]

{ #category : #'engine' }
APHYCBindings >> createSimpleRigidBodyFrom_engine: engine mass: mass motion_state: motion_state collision_shape: collision_shape local_inertia: local_inertia [
	^ self ffiCall: #(aphy_collision_object* aphyCreateSimpleRigidBodyFrom (aphy_engine* engine , aphy_scalar mass , aphy_motion_state* motion_state , aphy_collision_shape* collision_shape , aphy_vector3* local_inertia) )
]

{ #category : #'engine' }
APHYCBindings >> createGhostObject_engine: engine [
	^ self ffiCall: #(aphy_collision_object* aphyCreateGhostObject (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> createPairCachingGhostObject_engine: engine [
	^ self ffiCall: #(aphy_collision_object* aphyCreatePairCachingGhostObject (aphy_engine* engine) )
]

{ #category : #'engine' }
APHYCBindings >> createKinematicCharacterController_engine: engine ghost_object: ghost_object convex_shape: convex_shape step_height: step_height up_axis: up_axis [
	^ self ffiCall: #(aphy_character_controller* aphyCreateKinematicCharacterController (aphy_engine* engine , aphy_collision_object* ghost_object , aphy_collision_shape* convex_shape , aphy_scalar step_height , aphy_axis up_axis) )
]

{ #category : #'collision_configuration' }
APHYCBindings >> addReference_collision_configuration: collision_configuration [
	^ self ffiCall: #(aphy_error aphyAddCollisionConfigurationReference (aphy_collision_configuration* collision_configuration) )
]

{ #category : #'collision_configuration' }
APHYCBindings >> release_collision_configuration: collision_configuration [
	^ self ffiCall: #(aphy_error aphyReleaseCollisionConfiguration (aphy_collision_configuration* collision_configuration) )
]

{ #category : #'collision_dispatcher' }
APHYCBindings >> addReference_collision_dispatcher: collision_dispatcher [
	^ self ffiCall: #(aphy_error aphyAddCollisionDispatcherReference (aphy_collision_dispatcher* collision_dispatcher) )
]

{ #category : #'collision_dispatcher' }
APHYCBindings >> release_collision_dispatcher: collision_dispatcher [
	^ self ffiCall: #(aphy_error aphyReleaseCollisionDispatcher (aphy_collision_dispatcher* collision_dispatcher) )
]

{ #category : #'broadphase' }
APHYCBindings >> addReference_broadphase: broadphase [
	^ self ffiCall: #(aphy_error aphyAddBroadphaseReference (aphy_broadphase* broadphase) )
]

{ #category : #'broadphase' }
APHYCBindings >> release_broadphase: broadphase [
	^ self ffiCall: #(aphy_error aphyReleaseBroadphaseReference (aphy_broadphase* broadphase) )
]

{ #category : #'constraint_solver' }
APHYCBindings >> addReference_constraint_solver: constraint_solver [
	^ self ffiCall: #(aphy_error aphyAddConstraintSolverReference (aphy_constraint_solver* constraint_solver) )
]

{ #category : #'constraint_solver' }
APHYCBindings >> release_constraint_solver: constraint_solver [
	^ self ffiCall: #(aphy_error aphyReleaseConstraintSolverReference (aphy_constraint_solver* constraint_solver) )
]

{ #category : #'world' }
APHYCBindings >> addReference_world: world [
	^ self ffiCall: #(aphy_error aphyAddWorldReference (aphy_world* world) )
]

{ #category : #'world' }
APHYCBindings >> release_world: world [
	^ self ffiCall: #(aphy_error aphyReleaseWorldReference (aphy_world* world) )
]

{ #category : #'world' }
APHYCBindings >> getNumberOfCollisionObject_world: world [
	^ self ffiCall: #(aphy_uint aphyGetNumberOfCollisionObject (aphy_world* world) )
]

{ #category : #'world' }
APHYCBindings >> getNumberOfConstraints_world: world [
	^ self ffiCall: #(aphy_uint aphyGetNumberOfConstraints (aphy_world* world) )
]

{ #category : #'world' }
APHYCBindings >> addCollisionObject_world: world object: object collision_filter_group: collision_filter_group collision_filter_mask: collision_filter_mask [
	^ self ffiCall: #(aphy_error aphyAddCollisionObject (aphy_world* world , aphy_collision_object* object , aphy_short collision_filter_group , aphy_short collision_filter_mask) )
]

{ #category : #'world' }
APHYCBindings >> removeCollisionObject_world: world object: object [
	^ self ffiCall: #(aphy_error aphyRemoveCollisionObject (aphy_world* world , aphy_collision_object* object) )
]

{ #category : #'world' }
APHYCBindings >> addRigidBody_world: world object: object [
	^ self ffiCall: #(aphy_error aphyAddRigidBody (aphy_world* world , aphy_collision_object* object) )
]

{ #category : #'world' }
APHYCBindings >> removeRigidBody_world: world object: object [
	^ self ffiCall: #(aphy_error aphyRemoveRigidBody (aphy_world* world , aphy_collision_object* object) )
]

{ #category : #'world' }
APHYCBindings >> addCharacterController_world: world character: character [
	^ self ffiCall: #(aphy_error aphyAddCharacterController (aphy_world* world , aphy_character_controller* character) )
]

{ #category : #'world' }
APHYCBindings >> removeCharacterController_world: world character: character [
	^ self ffiCall: #(aphy_error aphyRemoveCharacterController (aphy_world* world , aphy_character_controller* character) )
]

{ #category : #'world' }
APHYCBindings >> addRigidBodyWithFilter_world: world object: object collision_filter_group: collision_filter_group collision_filter_mask: collision_filter_mask [
	^ self ffiCall: #(aphy_error aphyAddRigidBodyWithFilter (aphy_world* world , aphy_collision_object* object , aphy_short collision_filter_group , aphy_short collision_filter_mask) )
]

{ #category : #'world' }
APHYCBindings >> stepSimulation_world: world time_step: time_step max_sub_steps: max_sub_steps fixed_time_step: fixed_time_step [
	^ self ffiCall: #(aphy_error aphyStepSimulation (aphy_world* world , aphy_scalar time_step , aphy_int max_sub_steps , aphy_scalar fixed_time_step) )
]

{ #category : #'world' }
APHYCBindings >> setGravity_world: world x: x y: y z: z [
	^ self ffiCall: #(aphy_error aphySetGravity (aphy_world* world , aphy_scalar x , aphy_scalar y , aphy_scalar z) )
]

{ #category : #'world' }
APHYCBindings >> encodeDebugDrawing_world: world [
	^ self ffiCall: #(aphy_size aphyEncodeDebugDrawing (aphy_world* world) )
]

{ #category : #'world' }
APHYCBindings >> getDebugDrawingData_world: world buffer_size: buffer_size buffer: buffer [
	^ self ffiCall: #(aphy_error aphyGetDebugDrawingData (aphy_world* world , aphy_size buffer_size , aphy_pointer buffer) )
]

{ #category : #'character_controller' }
APHYCBindings >> addReference_character_controller: character_controller [
	^ self ffiCall: #(aphy_error aphyAddCharacterControllerReference (aphy_character_controller* character_controller) )
]

{ #category : #'character_controller' }
APHYCBindings >> release_character_controller: character_controller [
	^ self ffiCall: #(aphy_error aphyReleaseCharacterControllerReference (aphy_character_controller* character_controller) )
]

{ #category : #'character_controller' }
APHYCBindings >> setWalkDirection_character_controller: character_controller direction: direction [
	^ self ffiCall: #(aphy_error aphySetCharacterControllerWalkDirection (aphy_character_controller* character_controller , aphy_vector3 direction) )
]

{ #category : #'character_controller' }
APHYCBindings >> setWalkDirectionFrom_character_controller: character_controller direction: direction [
	^ self ffiCall: #(aphy_error aphySetCharacterControllerWalkDirectionFrom (aphy_character_controller* character_controller , aphy_vector3* direction) )
]

{ #category : #'character_controller' }
APHYCBindings >> setVelocityForTimeInterval_character_controller: character_controller velocity: velocity time_interval: time_interval [
	^ self ffiCall: #(aphy_error aphySetCharacterControllerVelocityForTimeInterval (aphy_character_controller* character_controller , aphy_vector3 velocity , aphy_scalar time_interval) )
]

{ #category : #'character_controller' }
APHYCBindings >> setVelocityForTimeIntervalFrom_character_controller: character_controller velocity: velocity time_interval: time_interval [
	^ self ffiCall: #(aphy_error aphySetCharacterControllerVelocityForTimeIntervalFrom (aphy_character_controller* character_controller , aphy_vector3* velocity , aphy_scalar time_interval) )
]

{ #category : #'character_controller' }
APHYCBindings >> warp_character_controller: character_controller origin: origin [
	^ self ffiCall: #(aphy_error aphyWarpCharacterController (aphy_character_controller* character_controller , aphy_vector3 origin) )
]

{ #category : #'character_controller' }
APHYCBindings >> warpWithOriginFrom_character_controller: character_controller origin: origin [
	^ self ffiCall: #(aphy_error aphyWarpCharacterControllerWithOriginFrom (aphy_character_controller* character_controller , aphy_vector3* origin) )
]

{ #category : #'character_controller' }
APHYCBindings >> canJump_character_controller: character_controller [
	^ self ffiCall: #(aphy_bool aphyCanCharacterControllerJump (aphy_character_controller* character_controller) )
]

{ #category : #'character_controller' }
APHYCBindings >> jump_character_controller: character_controller [
	^ self ffiCall: #(aphy_error aphyCharacterControllerJump (aphy_character_controller* character_controller) )
]

{ #category : #'character_controller' }
APHYCBindings >> isOnGround_character_controller: character_controller [
	^ self ffiCall: #(aphy_bool aphyIsCharacterControllerOnGround (aphy_character_controller* character_controller) )
]

{ #category : #'character_controller' }
APHYCBindings >> setMaxJumpHeight_character_controller: character_controller height: height [
	^ self ffiCall: #(aphy_error aphySetCharacterMaxJumpHeight (aphy_character_controller* character_controller , aphy_scalar height) )
]

{ #category : #'character_controller' }
APHYCBindings >> setJumpSpeed_character_controller: character_controller speed: speed [
	^ self ffiCall: #(aphy_error aphySetCharacterJumpSpeed (aphy_character_controller* character_controller , aphy_scalar speed) )
]

{ #category : #'character_controller' }
APHYCBindings >> setGravity_character_controller: character_controller gravity: gravity [
	^ self ffiCall: #(aphy_error aphySetCharacterGravity (aphy_character_controller* character_controller , aphy_scalar gravity) )
]

{ #category : #'collision_object' }
APHYCBindings >> addReference_collision_object: collision_object [
	^ self ffiCall: #(aphy_error aphyAddCollisionObjectReference (aphy_collision_object* collision_object) )
]

{ #category : #'collision_object' }
APHYCBindings >> release_collision_object: collision_object [
	^ self ffiCall: #(aphy_error aphyReleaseCollisionObjectReference (aphy_collision_object* collision_object) )
]

{ #category : #'collision_object' }
APHYCBindings >> getTransform_collision_object: collision_object [
	^ self ffiCall: #(aphy_transform aphyGetCollisionObjectTransform (aphy_collision_object* collision_object) )
]

{ #category : #'collision_object' }
APHYCBindings >> getTransformInto_collision_object: collision_object result: result [
	^ self ffiCall: #(aphy_error aphyGetCollisionObjectTransformInto (aphy_collision_object* collision_object , aphy_transform* result) )
]

{ #category : #'collision_object' }
APHYCBindings >> getTranslation_collision_object: collision_object [
	^ self ffiCall: #(aphy_vector3 aphyGetCollisionObjectTranslation (aphy_collision_object* collision_object) )
]

{ #category : #'collision_object' }
APHYCBindings >> getTranslationInto_collision_object: collision_object result: result [
	^ self ffiCall: #(aphy_error aphyGetCollisionObjectTranslationInto (aphy_collision_object* collision_object , aphy_vector3* result) )
]

{ #category : #'collision_object' }
APHYCBindings >> getMatrix_collision_object: collision_object [
	^ self ffiCall: #(aphy_matrix3x3 aphyGetCollisionObjectMatrix (aphy_collision_object* collision_object) )
]

{ #category : #'collision_object' }
APHYCBindings >> getMatrixInto_collision_object: collision_object result: result [
	^ self ffiCall: #(aphy_error aphyGetCollisionObjectMatrixInto (aphy_collision_object* collision_object , aphy_matrix3x3* result) )
]

{ #category : #'collision_object' }
APHYCBindings >> getQuaternion_collision_object: collision_object [
	^ self ffiCall: #(aphy_quaternion aphyGetCollisionObjectQuaternion (aphy_collision_object* collision_object) )
]

{ #category : #'collision_object' }
APHYCBindings >> getQuaternionInto_collision_object: collision_object result: result [
	^ self ffiCall: #(aphy_error aphyGetCollisionObjectQuaternionInto (aphy_collision_object* collision_object , aphy_quaternion* result) )
]

{ #category : #'collision_object' }
APHYCBindings >> setTransform_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectTransform (aphy_collision_object* collision_object , aphy_transform value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setTransformFrom_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectTransformFrom (aphy_collision_object* collision_object , aphy_transform* value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setTranslation_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectTranslation (aphy_collision_object* collision_object , aphy_vector3 value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setTranslationFrom_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectTranslationFrom (aphy_collision_object* collision_object , aphy_vector3* value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setMatrix_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectMatrix (aphy_collision_object* collision_object , aphy_matrix3x3 value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setMatrixFrom_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectMatrixFrom (aphy_collision_object* collision_object , aphy_matrix3x3* value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setQuaternion_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectQuaternion (aphy_collision_object* collision_object , aphy_quaternion value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setQuaternion_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectQuaternionFrom (aphy_collision_object* collision_object , aphy_quaternion* value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setCollisionShape_collision_object: collision_object shape: shape [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectShape (aphy_collision_object* collision_object , aphy_collision_shape* shape) )
]

{ #category : #'collision_object' }
APHYCBindings >> setHasContactResponse_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectHasContactResponse (aphy_collision_object* collision_object , aphy_bool value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setIsStaticObject_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectIsStatic (aphy_collision_object* collision_object , aphy_bool value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setIsKinematicObject_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectIsKinematicObject (aphy_collision_object* collision_object , aphy_bool value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setIsCharacterObject_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectIsCharacterObject (aphy_collision_object* collision_object , aphy_bool value) )
]

{ #category : #'collision_object' }
APHYCBindings >> setDebugDrawingEnabled_collision_object: collision_object value: value [
	^ self ffiCall: #(aphy_error aphySetCollisionObjectDebugDrawingEnabled (aphy_collision_object* collision_object , aphy_bool value) )
]

{ #category : #'collision_object' }
APHYCBindings >> getOverlappingObjectCount_collision_object: collision_object [
	^ self ffiCall: #(aphy_size aphyGetGhostCollisionObjectOverlappingObjectCount (aphy_collision_object* collision_object) )
]

{ #category : #'collision_object' }
APHYCBindings >> getOverlappingObject_collision_object: collision_object index: index [
	^ self ffiCall: #(aphy_collision_object* aphyGetGhostCollisionObjectOverlappingObject (aphy_collision_object* collision_object , aphy_size index) )
]

{ #category : #'collision_shape' }
APHYCBindings >> addReference_collision_shape: collision_shape [
	^ self ffiCall: #(aphy_error aphyAddCollisionShapeReference (aphy_collision_shape* collision_shape) )
]

{ #category : #'collision_shape' }
APHYCBindings >> release_collision_shape: collision_shape [
	^ self ffiCall: #(aphy_error aphyReleaseCollisionShapeReference (aphy_collision_shape* collision_shape) )
]

{ #category : #'collision_shape' }
APHYCBindings >> setMargin_collision_shape: collision_shape margin: margin [
	^ self ffiCall: #(aphy_error aphySetShapeMargin (aphy_collision_shape* collision_shape , aphy_scalar margin) )
]

{ #category : #'collision_shape' }
APHYCBindings >> getMargin_collision_shape: collision_shape [
	^ self ffiCall: #(aphy_scalar aphyGetShapeMargin (aphy_collision_shape* collision_shape) )
]

{ #category : #'collision_shape' }
APHYCBindings >> computeLocalInertia_collision_shape: collision_shape mass: mass [
	^ self ffiCall: #(aphy_vector3 aphyComputeLocalInertia (aphy_collision_shape* collision_shape , aphy_scalar mass) )
]

{ #category : #'collision_shape' }
APHYCBindings >> computeLocalInertiaInto_collision_shape: collision_shape mass: mass result: result [
	^ self ffiCall: #(aphy_error aphyComputeLocalInertiaInto (aphy_collision_shape* collision_shape , aphy_scalar mass , aphy_vector3* result) )
]

{ #category : #'collision_shape' }
APHYCBindings >> addLocalShapeWithTransform_collision_shape: collision_shape shape: shape transform: transform [
	^ self ffiCall: #(aphy_error aphyAddLocalShapeWithTransform (aphy_collision_shape* collision_shape , aphy_collision_shape* shape , aphy_transform transform) )
]

{ #category : #'collision_shape' }
APHYCBindings >> addLocalShapeWithTransformFrom_collision_shape: collision_shape shape: shape transform: transform [
	^ self ffiCall: #(aphy_error aphyAddLocalShapeWithTransformFrom (aphy_collision_shape* collision_shape , aphy_collision_shape* shape , aphy_transform* transform) )
]

{ #category : #'motion_state' }
APHYCBindings >> addReference_motion_state: motion_state [
	^ self ffiCall: #(aphy_error aphyAddMotionStateReference (aphy_motion_state* motion_state) )
]

{ #category : #'motion_state' }
APHYCBindings >> release_motion_state: motion_state [
	^ self ffiCall: #(aphy_error aphyReleaseMotionStateReference (aphy_motion_state* motion_state) )
]

{ #category : #'motion_state' }
APHYCBindings >> getTransform_motion_state: motion_state [
	^ self ffiCall: #(aphy_transform aphyGetMotionStateTransform (aphy_motion_state* motion_state) )
]

{ #category : #'motion_state' }
APHYCBindings >> getTransformInto_motion_state: motion_state result: result [
	^ self ffiCall: #(aphy_error aphyGetMotionStateTransformInto (aphy_motion_state* motion_state , aphy_transform* result) )
]

{ #category : #'motion_state' }
APHYCBindings >> getTranslation_motion_state: motion_state [
	^ self ffiCall: #(aphy_vector3 aphyGetMotionStateTranslation (aphy_motion_state* motion_state) )
]

{ #category : #'motion_state' }
APHYCBindings >> getTranslationInto_motion_state: motion_state result: result [
	^ self ffiCall: #(aphy_error aphyGetMotionStateTranslationInto (aphy_motion_state* motion_state , aphy_vector3* result) )
]

{ #category : #'motion_state' }
APHYCBindings >> getMatrix_motion_state: motion_state [
	^ self ffiCall: #(aphy_matrix3x3 aphyGetMotionStateMatrix (aphy_motion_state* motion_state) )
]

{ #category : #'motion_state' }
APHYCBindings >> getMatrixInto_motion_state: motion_state result: result [
	^ self ffiCall: #(aphy_error aphyGetMotionStateMatrixInto (aphy_motion_state* motion_state , aphy_matrix3x3* result) )
]

{ #category : #'motion_state' }
APHYCBindings >> getQuaternion_motion_state: motion_state [
	^ self ffiCall: #(aphy_quaternion aphyGetMotionStateQuaternion (aphy_motion_state* motion_state) )
]

{ #category : #'motion_state' }
APHYCBindings >> getQuaternionInto_motion_state: motion_state result: result [
	^ self ffiCall: #(aphy_error aphyGetMotionStateQuaternionInto (aphy_motion_state* motion_state , aphy_quaternion* result) )
]

{ #category : #'motion_state' }
APHYCBindings >> setTransform_motion_state: motion_state value: value [
	^ self ffiCall: #(aphy_error aphySetMotionStateTransform (aphy_motion_state* motion_state , aphy_transform value) )
]

{ #category : #'motion_state' }
APHYCBindings >> setTransformFrom_motion_state: motion_state value: value [
	^ self ffiCall: #(aphy_error aphySetMotionStateTransformFrom (aphy_motion_state* motion_state , aphy_transform* value) )
]

{ #category : #'motion_state' }
APHYCBindings >> setTranslation_motion_state: motion_state value: value [
	^ self ffiCall: #(aphy_error aphySetMotionStateTranslation (aphy_motion_state* motion_state , aphy_vector3 value) )
]

{ #category : #'motion_state' }
APHYCBindings >> setTranslationFrom_motion_state: motion_state value: value [
	^ self ffiCall: #(aphy_error aphySetMotionStateTranslationFrom (aphy_motion_state* motion_state , aphy_vector3* value) )
]

{ #category : #'motion_state' }
APHYCBindings >> setMatrix_motion_state: motion_state value: value [
	^ self ffiCall: #(aphy_error aphySetMotionStateMatrix (aphy_motion_state* motion_state , aphy_matrix3x3 value) )
]

{ #category : #'motion_state' }
APHYCBindings >> setMatrixFrom_motion_state: motion_state value: value [
	^ self ffiCall: #(aphy_error aphySetMotionStateMatrixFrom (aphy_motion_state* motion_state , aphy_matrix3x3* value) )
]

{ #category : #'motion_state' }
APHYCBindings >> setQuaternion_motion_state: motion_state value: value [
	^ self ffiCall: #(aphy_error aphySetMotionStateQuaternion (aphy_motion_state* motion_state , aphy_quaternion value) )
]

{ #category : #'motion_state' }
APHYCBindings >> setQuaternionFrom_motion_state: motion_state value: value [
	^ self ffiCall: #(aphy_error aphySetMotionStateQuaternionFrom (aphy_motion_state* motion_state , aphy_quaternion* value) )
]

{ #category : #'global c functions' }
APHYCBindings >> getEngines_numengines: numengines engines: engines ret_numengines: ret_numengines [
	^ self ffiCall: #(aphy_error aphyGetEngines (aphy_size numengines , aphy_engine* engines , aphy_size* ret_numengines) )
]

