Class {
	#name : #APHYCharacterController,
	#superclass : #APHYInterface,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'wrappers' }
APHYCharacterController >> addReference [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addReference_character_controller: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> primitiveRelease [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance release_character_controller: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> setWalkDirection: direction [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setWalkDirection_character_controller: (self validHandle) direction: direction.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> setWalkDirectionFrom: direction [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setWalkDirectionFrom_character_controller: (self validHandle) direction: direction.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> setVelocityForTimeInterval: velocity time_interval: time_interval [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setVelocityForTimeInterval_character_controller: (self validHandle) velocity: velocity time_interval: time_interval.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> setVelocityForTimeIntervalFrom: velocity time_interval: time_interval [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setVelocityForTimeIntervalFrom_character_controller: (self validHandle) velocity: velocity time_interval: time_interval.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> warp: origin [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance warp_character_controller: (self validHandle) origin: origin.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> warpWithOriginFrom: origin [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance warpWithOriginFrom_character_controller: (self validHandle) origin: origin.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> canJump [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance canJump_character_controller: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> jump [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance jump_character_controller: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> isOnGround [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance isOnGround_character_controller: (self validHandle).
	^ resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> setMaxJumpHeight: height [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setMaxJumpHeight_character_controller: (self validHandle) height: height.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> setJumpSpeed: speed [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setJumpSpeed_character_controller: (self validHandle) speed: speed.
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCharacterController >> setGravity: gravity [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance setGravity_character_controller: (self validHandle) gravity: gravity.
	self checkErrorCode: resultValue_
]

