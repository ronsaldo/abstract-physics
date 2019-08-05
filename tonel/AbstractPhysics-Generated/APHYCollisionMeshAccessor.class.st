Class {
	#name : #APHYCollisionMeshAccessor,
	#pools : [
		'APHYConstants',
		'APHYTypes'
	],
	#superclass : #FFIExternalStructure,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'definition' }
APHYCollisionMeshAccessor class >> fieldsDesc [
	"
	self rebuildFieldAccessors
	"
    ^ #(
		 aphy_pointer vertices;
		 aphy_size vertex_offset;
		 aphy_size vertex_stride;
		 aphy_size vertex_count;
		 aphy_pointer indices;
		 aphy_size index_offset;
		 aphy_size index_stride;
		 aphy_size index_count;
	)
]

