Class {
	#name : #APHYCBindingsBase,
	#superclass : #FFILibrary,
	#category : #'AbstractPhysics-Core'
}

{ #category : #'accessing platform' }
APHYCBindingsBase >> macModuleName [
	^ 'libAPhyBullet.dylib'
]

{ #category : #'accessing platform' }
APHYCBindingsBase >> unixModuleName [
	^ 'libAPhyBullet.so'
]

{ #category : #'accessing platform' }
APHYCBindingsBase >> win32ModuleName [
	^ 'APhyBullet.dll'
]
