Class {
	#name : #APHYMotionState,
	#superclass : #APHYInterface,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'wrappers' }
APHYMotionState >> addReference [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addReference_motion_state: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> primitiveRelease [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance release_motion_state: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> getTransform [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getTransform_motion_state: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> getTransformInto: result [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getTransformInto_motion_state: (self validHandle) result: result.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> getTranslation [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getTranslation_motion_state: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> getTranslationInto: result [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getTranslationInto_motion_state: (self validHandle) result: result.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> getMatrix [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getMatrix_motion_state: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> getMatrixInto: result [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getMatrixInto_motion_state: (self validHandle) result: result.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> getQuaternion [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getQuaternion_motion_state: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> getQuaternionInto: result [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getQuaternionInto_motion_state: (self validHandle) result: result.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> setTransform: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setTransform_motion_state: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> setTransformFrom: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setTransformFrom_motion_state: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> setTranslation: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setTranslation_motion_state: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> setTranslationFrom: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setTranslationFrom_motion_state: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> setMatrix: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setMatrix_motion_state: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> setMatrixFrom: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setMatrixFrom_motion_state: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> setQuaternion: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setQuaternion_motion_state: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYMotionState >> setQuaternionFrom: value [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setQuaternionFrom_motion_state: (self validHandle) value: value.
	self checkErrorCode: resultValue_
]

