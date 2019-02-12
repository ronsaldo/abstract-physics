Class {
	#name : #APHYWorld,
	#superclass : #APHYInterface,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'wrappers' }
APHYWorld >> addReference [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addReference_world: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> primitiveRelease [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance release_world: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> getNumberOfCollisionObject [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getNumberOfCollisionObject_world: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> getNumberOfConstraints [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getNumberOfConstraints_world: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> addCollisionObject: object collision_filter_group: collision_filter_group collision_filter_mask: collision_filter_mask [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addCollisionObject_world: (self validHandle) object: (self validHandleOf: object) collision_filter_group: collision_filter_group collision_filter_mask: collision_filter_mask.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> removeCollisionObject: object [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance removeCollisionObject_world: (self validHandle) object: (self validHandleOf: object).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> addRigidBody: object [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addRigidBody_world: (self validHandle) object: (self validHandleOf: object).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> removeRigidBody: object [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance removeRigidBody_world: (self validHandle) object: (self validHandleOf: object).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> addCharacterController: character [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addCharacterController_world: (self validHandle) character: (self validHandleOf: character).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> removeCharacterController: character [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance removeCharacterController_world: (self validHandle) character: (self validHandleOf: character).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> addRigidBodyWithFilter: object collision_filter_group: collision_filter_group collision_filter_mask: collision_filter_mask [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addRigidBodyWithFilter_world: (self validHandle) object: (self validHandleOf: object) collision_filter_group: collision_filter_group collision_filter_mask: collision_filter_mask.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> stepSimulation: time_step max_sub_steps: max_sub_steps fixed_time_step: fixed_time_step [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance stepSimulation_world: (self validHandle) time_step: time_step max_sub_steps: max_sub_steps fixed_time_step: fixed_time_step.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> setGravity: x y: y z: z [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setGravity_world: (self validHandle) x: x y: y z: z.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> encodeDebugDrawing [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance encodeDebugDrawing_world: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYWorld >> getDebugDrawingData: buffer_size buffer: buffer [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getDebugDrawingData_world: (self validHandle) buffer_size: buffer_size buffer: buffer.
	self checkErrorCode: resultValue_
]

