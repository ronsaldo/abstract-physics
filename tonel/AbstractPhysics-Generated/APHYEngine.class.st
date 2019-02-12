Class {
	#name : #APHYEngine,
	#superclass : #APHYInterface,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'wrappers' }
APHYEngine >> addReference [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addReference_engine: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> primitiveRelease [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance release_engine: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> getName [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getName_engine: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> getVersion [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getVersion_engine: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createDefaultCollisionConfiguration [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createDefaultCollisionConfiguration_engine: (self validHandle).
	^ APHYCollisionConfiguration forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createDefaultCollisionDispatcher: collision_configuration [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createDefaultCollisionDispatcher_engine: (self validHandle) collision_configuration: (self validHandleOf: collision_configuration).
	^ APHYCollisionDispatcher forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createDefaultBroadphase [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createDefaultBroadphase_engine: (self validHandle).
	^ APHYBroadphase forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createDefaultConstraintSolver [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createDefaultConstraintSolver_engine: (self validHandle).
	^ APHYConstraintSolver forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createDefaultMotionState [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createDefaultMotionState_engine: (self validHandle).
	^ APHYMotionState forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createDynamicsWorld: collision_dispatcher broadphase: broadphase constraint_solver: constraint_solver collision_configuration: collision_configuration [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createDynamicsWorld_engine: (self validHandle) collision_dispatcher: (self validHandleOf: collision_dispatcher) broadphase: (self validHandleOf: broadphase) constraint_solver: (self validHandleOf: constraint_solver) collision_configuration: (self validHandleOf: collision_configuration).
	^ APHYWorld forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createBoxShape: half_width half_height: half_height half_depth: half_depth [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createBoxShape_engine: (self validHandle) half_width: half_width half_height: half_height half_depth: half_depth.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createCylinderX: half_width half_height: half_height half_depth: half_depth [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createCylinderX_engine: (self validHandle) half_width: half_width half_height: half_height half_depth: half_depth.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createCylinderY: half_width half_height: half_height half_depth: half_depth [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createCylinderY_engine: (self validHandle) half_width: half_width half_height: half_height half_depth: half_depth.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createCylinderZ: half_width half_height: half_height half_depth: half_depth [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createCylinderZ_engine: (self validHandle) half_width: half_width half_height: half_height half_depth: half_depth.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createConvexHullShape: coordinates num_points: num_points stride: stride [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createConvexHullShape_engine: (self validHandle) coordinates: coordinates num_points: num_points stride: stride.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createCapsuleShapeX: radius height: height [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createCapsuleShapeX_engine: (self validHandle) radius: radius height: height.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createCapsuleShapeY: radius height: height [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createCapsuleShapeY_engine: (self validHandle) radius: radius height: height.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createCapsuleShapeZ: radius height: height [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createCapsuleShapeZ_engine: (self validHandle) radius: radius height: height.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createCompoundShape [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createCompoundShape_engine: (self validHandle).
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createConeX: radius height: height [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createConeX_engine: (self validHandle) radius: radius height: height.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createConeY: radius height: height [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createConeY_engine: (self validHandle) radius: radius height: height.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createConeZ: radius height: height [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createConeZ_engine: (self validHandle) radius: radius height: height.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createEmptyShape [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createEmptyShape_engine: (self validHandle).
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createHeightfieldTerrainShape: height_stick_width height_stick_length: height_stick_length heightfield_data: heightfield_data height_scale: height_scale min_height: min_height max_height: max_height up_axis: up_axis height_data_type: height_data_type flip_quad_edges: flip_quad_edges local_scale_x: local_scale_x local_scale_y: local_scale_y local_scale_z: local_scale_z [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createHeightfieldTerrainShape_engine: (self validHandle) height_stick_width: height_stick_width height_stick_length: height_stick_length heightfield_data: heightfield_data height_scale: height_scale min_height: min_height max_height: max_height up_axis: up_axis height_data_type: height_data_type flip_quad_edges: flip_quad_edges local_scale_x: local_scale_x local_scale_y: local_scale_y local_scale_z: local_scale_z.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createSphere: radius [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createSphere_engine: (self validHandle) radius: radius.
	^ APHYCollisionShape forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createSimpleRigidBody: mass motion_state: motion_state collision_shape: collision_shape local_inertia: local_inertia [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createSimpleRigidBody_engine: (self validHandle) mass: mass motion_state: (self validHandleOf: motion_state) collision_shape: (self validHandleOf: collision_shape) local_inertia: local_inertia.
	^ APHYCollisionObject forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createSimpleRigidBodyFrom: mass motion_state: motion_state collision_shape: collision_shape local_inertia: local_inertia [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createSimpleRigidBodyFrom_engine: (self validHandle) mass: mass motion_state: (self validHandleOf: motion_state) collision_shape: (self validHandleOf: collision_shape) local_inertia: local_inertia.
	^ APHYCollisionObject forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createGhostObject [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createGhostObject_engine: (self validHandle).
	^ APHYCollisionObject forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createPairCachingGhostObject [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createPairCachingGhostObject_engine: (self validHandle).
	^ APHYCollisionObject forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYEngine >> createKinematicCharacterController: ghost_object convex_shape: convex_shape step_height: step_height up_axis: up_axis [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance createKinematicCharacterController_engine: (self validHandle) ghost_object: (self validHandleOf: ghost_object) convex_shape: (self validHandleOf: convex_shape) step_height: step_height up_axis: up_axis.
	^ APHYCharacterController forHandle: resultValue_
]

