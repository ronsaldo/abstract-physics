Class {
	#name : #APHYBroadphase,
	#superclass : #APHYInterface,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'wrappers' }
APHYBroadphase >> addReference [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addReference_broadphase: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYBroadphase >> primitiveRelease [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance release_broadphase: (self validHandle).
	self checkErrorCode: resultValue_
]

