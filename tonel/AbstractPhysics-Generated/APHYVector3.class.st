Class {
	#name : #APHYVector3,
	#pools : [
		'APHYConstants',
		'APHYTypes'
	],
	#superclass : #FFIExternalStructure,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'definition' }
APHYVector3 class >> fieldsDesc [
	"
	self rebuildFieldAccessors
	"
	^ #(
		 aphy_scalar x;
		 aphy_scalar y;
		 aphy_scalar z;
		 aphy_scalar pad;
		)
]

