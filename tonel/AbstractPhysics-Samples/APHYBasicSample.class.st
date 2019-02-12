Class {
	#name : #APHYBasicSample,
	#superclass : #Object,
	#instVars : [
		'engine',
		'collisionConfiguration',
		'collisionDispatcher',
		'broadphase',
		'constraintSolver',
		'world'
	],
	#category : #'AbstractPhysics-Samples'
}

{ #category : #'as yet unclassified' }
APHYBasicSample >> createWorld [
	engine := APHY firstEngine.

	collisionConfiguration := engine createDefaultCollisionConfiguration.
	collisionDispatcher := engine createDefaultCollisionDispatcher: collisionConfiguration.

	broadphase := engine createDefaultBroadphase.
	constraintSolver := engine createDefaultConstraintSolver.

	world := engine createDynamicsWorld: collisionDispatcher broadphase: broadphase constraint_solver: 	constraintSolver collision_configuration: collisionConfiguration.

]
