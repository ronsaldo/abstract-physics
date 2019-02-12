Class {
	#name : #APHY,
	#superclass : #APHYBindingsCommon,
	#instVars : [
		'session',
		'mainPlatform',
		'mainDevice'
	],
	#classVars : [
		'UniqueInstance'
	],
	#category : #'AbstractPhysics-Core'
}

{ #category : #'platform querying' }
APHY class >> engines [
	^ self uniqueInstance engines
]

{ #category : #'platform querying' }
APHY class >> firstEngine [
	^ self uniqueInstance firstEngine
]

{ #category : #'platform querying' }
APHY class >> uniqueInstance [
	UniqueInstance ifNil: [ UniqueInstance := self new. ].
	^ UniqueInstance
]

{ #category : #'main device' }
APHY >> checkSession [
	(session ~~ Smalltalk session or: [mainDevice isNil]) ifTrue: [
		self openMainDevice.
		session := Smalltalk session.
	].

]

{ #category : #'platform querying' }
APHY >> engineCount [
	| engineCount |
	engineCount := ExternalAddress allocate: 4.
	[ 
		self getEngines: 0 engines: nil ret_numengines: engineCount.
		^ engineCount unsignedLongAt: 1 
	] ensure: [ engineCount free ].
]

{ #category : #'platform querying' }
APHY >> engines [
	| engineCount enginePointers engineCountRet pointerSize |
	engineCount := self engineCount.
	engineCountRet := ExternalAddress allocate: 4.
	[ 
		pointerSize := Smalltalk wordSize.
		enginePointers := ExternalAddress allocate: pointerSize * engineCount.
		[
			self getEngines: engineCount engines: enginePointers ret_numengines: engineCountRet.
			engineCount := engineCountRet unsignedLongAt: 1.
			^ (1 to: engineCount) collect: [ :i | (Smalltalk at: #APHYEngine) forManualHandle: (enginePointers pointerAt: i*pointerSize - 3)]
		] ensure: [ enginePointers free ]
	] ensure: [ engineCountRet free ].
]

{ #category : #'platform querying' }
APHY >> firstEngine [
	^ self engines first
]
