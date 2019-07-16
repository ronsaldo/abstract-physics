Class {
	#name : #APHYTransform,
	#pools : [
		'APHYConstants',
		'APHYTypes'
	],
	#superclass : #FFIExternalStructure,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'definition' }
APHYTransform class >> fieldsDesc [
	"
	self rebuildFieldAccessors
	"
    ^ #(
		 aphy_matrix3x3 rotation;
		 aphy_vector3 origin;
	)
]

