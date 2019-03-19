Class {
	#name : #APHYCBindingsBase,
	#superclass : #PhaNAPILibrary,
	#category : #'AbstractPhysics-Core'
}

{ #category : #'library downloading' }
APHYCBindingsBase >> fetchLibrary [
	PhaNAPIBintray fetchLibrary: 'APhyBullet' fromUser: 'ronsaldo' repository: 'abstract-physics' package: 'lib'.

]

{ #category : #'library downloading' }
APHYCBindingsBase >> findLibraryPath [
	^ PhaNAPI fullLibraryPathForCurrentPlatform: 'APhyBullet' package: 'abstract-physics'
]
