Class {
	#name : #APHYCollisionObject,
	#superclass : #APHYInterface,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'wrappers' }
APHYCollisionObject >> addReference [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addReference_collision_object: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> primitiveRelease [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance release_collision_object: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> getTransform [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getTransform_collision_object: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> getTransformInto: result [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getTransformInto_collision_object: (self validHandle) result: result.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> getTranslation [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getTranslation_collision_object: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> getTranslationInto: result [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getTranslationInto_collision_object: (self validHandle) result: result.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> getMatrix [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getMatrix_collision_object: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> getMatrixInto: result [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getMatrixInto_collision_object: (self validHandle) result: result.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> getQuaternion [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getQuaternion_collision_object: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> getQuaternionInto: result [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getQuaternionInto_collision_object: (self validHandle) result: result.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setTransform: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setTransform_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setTransformFrom: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setTransformFrom_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setTranslation: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setTranslation_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setTranslationFrom: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setTranslationFrom_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setMatrix: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setMatrix_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setMatrixFrom: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setMatrixFrom_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setQuaternion: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setQuaternion_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setQuaternion: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setQuaternion_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setCollisionShape: shape [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setCollisionShape_collision_object: (self validHandle) shape: (self validHandleOf: shape).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setHasContactResponse: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setHasContactResponse_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setIsStaticObject: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setIsStaticObject_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setIsKinematicObject: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setIsKinematicObject_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setIsCharacterObject: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setIsCharacterObject_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setDebugDrawingEnabled: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setDebugDrawingEnabled_collision_object: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> getOverlappingObjectCount [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getOverlappingObjectCount_collision_object: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> getOverlappingObject: index [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getOverlappingObject_collision_object: (self validHandle) index: index.
	^ APHYCollisionObject forHandle: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> activate [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance activate_collision_object: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setLinearVelocityFrom: velocity [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setLinearVelocityFrom_collision_object: (self validHandle) velocity: velocity.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionObject >> setAngularVelocityFrom: velocity [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setAngularVelocityFrom_collision_object: (self validHandle) velocity: velocity.
	self checkErrorCode: resultValue_
]

