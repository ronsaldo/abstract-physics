Class {
	#name : #APHYCollisionMeshCollection,
	#superclass : #APHYInterface,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'wrappers' }
APHYCollisionMeshCollection >> addReference [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addReference_collision_mesh_collection: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionMeshCollection >> primitiveRelease [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance release_collision_mesh_collection: (self validHandle).
	self checkErrorCode: resultValue_
]

{ #category : #'wrappers' }
APHYCollisionMeshCollection >> addCollisionMeshAccessor: accessor transform: transform [
	| resultValue_ |
	resultValue_ := APHYCBindings uniqueInstance addCollisionMeshAccessor_collision_mesh_collection: (self validHandle) accessor: accessor transform: transform.
	self checkErrorCode: resultValue_
]

