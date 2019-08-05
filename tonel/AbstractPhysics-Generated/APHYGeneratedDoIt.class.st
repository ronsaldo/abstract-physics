Class {
	#name : #APHYGeneratedDoIt,
	#superclass : #Object,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'initialization' }
APHYGeneratedDoIt class >> initializeConstants [
	<script>
	APHYTypes initialize.
	APHYConstants initialize.
]

{ #category : #'initialization' }
APHYGeneratedDoIt class >> initializeStructures [
	<script>
	APHYVector3 rebuildFieldAccessors.
	APHYQuaternion rebuildFieldAccessors.
	APHYMatrix3x3 rebuildFieldAccessors.
	APHYTransform rebuildFieldAccessors.
	APHYCollisionMeshAccessor rebuildFieldAccessors.
]

{ #category : #'initialization' }
APHYGeneratedDoIt class >> initializeBindings [
	<script>
	self initializeConstants.
	self initializeStructures.
]

