Extension { #name : #APHY }

{ #category : #'*AbstractPhysics-Generated' }
APHY >> getEngines: numengines engines: engines ret_numengines: ret_numengines [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance getEngines_numengines: numengines engines: engines ret_numengines: ret_numengines.
	self checkErrorCode: resultValue_
]

