Class {
	#name : #APHYCBindingsBase,
	#superclass : #PhaNAPILibrary,
	#category : #'AbstractPhysics-Core'
}

{ #category : #'library downloading' }
APHYCBindingsBase >> fetchLibrary [
	PhaNAPIGitHubRelease fetchLibrary: 'APhyBullet' fromIcebergRepoOfClass: self class

]

{ #category : #'library downloading' }
APHYCBindingsBase >> findLibraryPath [
	^ PhaNAPI fullLibraryPathForCurrentPlatform: 'APhyBullet' package: 'abstract-physics'
]
