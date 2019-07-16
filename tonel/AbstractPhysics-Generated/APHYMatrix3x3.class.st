Class {
	#name : #APHYMatrix3x3,
	#pools : [
		'APHYConstants',
		'APHYTypes'
	],
	#superclass : #FFIExternalStructure,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'definition' }
APHYMatrix3x3 class >> fieldsDesc [
	"
	self rebuildFieldAccessors
	"
    ^ #(
		 aphy_vector3 firstRow;
		 aphy_vector3 secondRow;
		 aphy_vector3 thirdRow;
	)
]

