Class {
	#name : #APHYQuaternion,
	#pools : [
		'APHYConstants',
		'APHYTypes'
	],
	#superclass : #FFIExternalStructure,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'definition' }
APHYQuaternion class >> fieldsDesc [
	"
	self rebuildFieldAccessors
	"
    ^ #(
		 aphy_scalar x;
		 aphy_scalar y;
		 aphy_scalar z;
		 aphy_scalar w;
	)
]

