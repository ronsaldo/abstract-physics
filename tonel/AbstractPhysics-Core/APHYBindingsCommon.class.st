Class {
	#name : #APHYBindingsCommon,
	#superclass : #Object,
	#classVars : [
		'ModuleName',
		'ModuleSession'
	],
	#category : #'AbstractPhysics-Core'
}

{ #category : #'as yet unclassified' }
APHYBindingsCommon class >> checkErrorCode: errorCode [
	errorCode < 0 ifTrue: [ 
		self error: 'Got an error from AGPU.'
	]
]

{ #category : #'as yet unclassified' }
APHYBindingsCommon class >> validHandleOf: anObject [
	^ anObject ifNil: [ ExternalAddress null ] ifNotNil: [ anObject validHandle ]
]

{ #category : #'error checking' }
APHYBindingsCommon >> checkErrorCode: errorCode [
	errorCode < 0 ifTrue: [ 
		Transcript show: 'Got an error from AGPU: ' , errorCode printString; cr.
		self error: 'Got an error from AGPU.'
	]
]

{ #category : #'freeing resources' }
APHYBindingsCommon >> release [
	self primitiveRelease.
]

{ #category : #accessing }
APHYBindingsCommon >> validHandleOf: anObject [
	^ anObject ifNotNil: [ anObject validHandle ]
]
