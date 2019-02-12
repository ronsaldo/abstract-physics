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
		spec 
			package: 'AbstractPhysics-Core';
			package: 'AbstractPhysics-Generated' with: [
				spec requires: #('AbstractPhysics-Core' ). ];
			package: 'AbstractPhysics-Samples' with: [
				spec requires: #('AbstractPhysics-Core' 'AbstractPhysics-Generated' ). ]. ].

]
