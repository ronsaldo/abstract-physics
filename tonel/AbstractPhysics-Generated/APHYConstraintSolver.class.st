Class {
	#name : #APHYConstraintSolver,
	#superclass : #APHYInterface,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'wrappers' }
APHYConstraintSolver >> addReference [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addReference_constraint_solver: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYConstraintSolver >> primitiveRelease [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance release_constraint_solver: (self validHandle).
	self checkErrorCode: resultValue_
]

