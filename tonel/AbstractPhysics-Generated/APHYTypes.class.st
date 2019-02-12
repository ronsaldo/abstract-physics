Class {
	#name : #APHYTypes,
	#classVars : [
		'aphy_enum',
		'aphy_string',
		'aphy_short',
		'aphy_int',
		'aphy_uint',
		'aphy_bitfield',
		'aphy_bool',
		'aphy_vector3',
		'aphy_quaternion',
		'aphy_size',
		'aphy_cstring_buffer',
		'aphy_scalar_type',
		'aphy_world',
		'aphy_cstring',
		'aphy_string_length',
		'aphy_pointer',
		'aphy_matrix3x3',
		'aphy_scalar',
		'aphy_collision_configuration',
		'aphy_byte',
		'aphy_sbyte',
		'aphy_string_buffer',
		'aphy_error',
		'aphy_engine',
		'aphy_transform',
		'aphy_constraint_solver',
		'aphy_collision_shape',
		'aphy_character_controller',
		'aphy_axis',
		'aphy_ushort',
		'aphy_collision_object',
		'aphy_collision_dispatcher',
		'aphy_broadphase',
		'aphy_motion_state',
		'aphy_debug_draw_opcode'
	],
	#superclass : #SharedPool,
	#category : 'AbstractPhysics-Generated'
}

{ #category : #'initialize' }
APHYTypes class >> initialize [
"
	self initialize
"
	super initialize.

	aphy_enum := #'int'.
	aphy_string := #'char*'.
	aphy_short := #'short'.
	aphy_int := #'int'.
	aphy_uint := #'uint'.
	aphy_bitfield := #'uint'.
	aphy_bool := #'int'.
	aphy_vector3 := APHYVector3.
	aphy_quaternion := APHYQuaternion.
	aphy_size := #'uint'.
	aphy_cstring_buffer := #'char*'.
	aphy_scalar_type := #int.
	aphy_world := #'void'.
	aphy_cstring := #'char*'.
	aphy_string_length := #'int'.
	aphy_pointer := #'void*'.
	aphy_matrix3x3 := APHYMatrix3x3.
	aphy_scalar := #'float'.
	aphy_collision_configuration := #'void'.
	aphy_byte := #'uchar'.
	aphy_sbyte := #'char'.
	aphy_string_buffer := #'char*'.
	aphy_error := #int.
	aphy_engine := #'void'.
	aphy_transform := APHYTransform.
	aphy_constraint_solver := #'void'.
	aphy_collision_shape := #'void'.
	aphy_character_controller := #'void'.
	aphy_axis := #int.
	aphy_ushort := #'ushort'.
	aphy_collision_object := #'void'.
	aphy_collision_dispatcher := #'void'.
	aphy_broadphase := #'void'.
	aphy_motion_state := #'void'.
	aphy_debug_draw_opcode := #int.
]

