Class {
	#name : #APHYCollisionDispatcher,
	#superclass : #APHYInterface,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'wrappers' }
APHYCollisionDispatcher >> addReference [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addReference_collision_dispatcher: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionDispatcher >> primitiveRelease [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance release_collision_dispatcher: (self validHandle).
	self checkErrorCode: resultValue_
]

