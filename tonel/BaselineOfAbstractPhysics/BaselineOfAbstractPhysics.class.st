Class {
	#name : #BaselineOfAbstractPhysics,
	#superclass : #BaselineOf,
	#category : #BaselineOfAbstractPhysics
}

{ #category : #baselines }
BaselineOfAbstractPhysics >> baseline: spec [
    <baseline>
	spec for: #'common' do: [
		spec blessing: #'baseline'.
		spec postLoadDoIt: #postLoadActions.
		spec 
			baseline: #'PhaNAPI' with: [
				spec
					repository: 'github://ronsaldo/phanapi/tonel' ].

		spec 
			package: #'AbstractPhysics-Core' with: [
				spec requires: #(#'PhaNAPI' ). ];
			package: #'AbstractPhysics-Generated' with: [
				spec requires: #(#'AbstractPhysics-Core' ). ];
			package: #'AbstractPhysics-Samples' with: [
				spec requires: #(#'AbstractPhysics-Core' #'AbstractPhysics-Generated' ). ]. ].

]

{ #category : #baselines }
BaselineOfAbstractPhysics >> postLoadActions [
	(Smalltalk globals at: #APHYCBindingsBase) fetchLibrary
]
