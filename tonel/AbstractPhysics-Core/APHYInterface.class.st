Class {
	#name : #APHYInterface,
	#superclass : #APHYBindingsCommon,
	#instVars : [
		'handle'
	],
	#category : #'AbstractPhysics-Core'
}

{ #category : #'instance creation' }
APHYInterface class >> forHandle: aHandle [
	(aHandle isNil or: [ aHandle isNull ]) ifTrue: [ ^ nil ].
	^ self new handle: aHandle; autoRelease; yourself
]

{ #category : #'instance creation' }
APHYInterface class >> forManualHandle: aHandle [
	^ self new handle: aHandle
]

{ #category : #finalization }
APHYInterface >> autoRelease [
	^ self class finalizationRegistry add: self
]

{ #category : #accessing }
APHYInterface >> finalize [
	(handle isNotNil and: [ handle isNull not ]) ifTrue: [
		self primitiveRelease
	]
	
]

{ #category : #accessing }
APHYInterface >> handle [
	^ handle
]

{ #category : #accessing }
APHYInterface >> handle: aHandle [
	handle := aHandle
]

{ #category : #testing }
APHYInterface >> isNull [
	^ handle isNil or: [ handle isNull ]
]

{ #category : #finalization }
APHYInterface >> release [
	self class finalizationRegistry remove: self.
	self primitiveRelease.
	handle := nil.
]

{ #category : #accessing }
APHYInterface >> validHandle [
	handle ifNil: [ self error: 'Trying to use an object with an invalid handle.' ].
	^ handle
]
