Class {
	#name : #APHYCollisionShape,
	#superclass : #APHYInterface,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'wrappers' }
APHYCollisionShape >> addReference [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addReference_collision_shape: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionShape >> primitiveRelease [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance release_collision_shape: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionShape >> setMargin: margin [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setMargin_collision_shape: (self validHandle) margin: margin.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionShape >> getMargin [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getMargin_collision_shape: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYCollisionShape >> computeLocalInertia: mass [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance computeLocalInertia_collision_shape: (self validHandle) mass: mass.
	^ resultValue_
]

{ #category : #'wrappers' }
APHYCollisionShape >> computeLocalInertiaInto: mass result: result [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance computeLocalInertiaInto_collision_shape: (self validHandle) mass: mass result: result.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionShape >> addLocalShapeWithTransform: shape transform: transform [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addLocalShapeWithTransform_collision_shape: (self validHandle) shape: (self validHandleOf: shape) transform: transform.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionShape >> addLocalShapeWithTransformFrom: shape transform: transform [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addLocalShapeWithTransformFrom_collision_shape: (self validHandle) shape: (self validHandleOf: shape) transform: transform.
	self checkErrorCode: resultValue_
]

