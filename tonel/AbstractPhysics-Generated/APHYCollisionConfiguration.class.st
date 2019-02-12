Class {
	#name : #APHYCollisionConfiguration,
	#superclass : #APHYInterface,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'wrappers' }
APHYCollisionConfiguration >> addReference [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addReference_collision_configuration: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionConfiguration >> primitiveRelease [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance release_collision_configuration: (self validHandle).
	self checkErrorCode: resultValue_
]

