/*
procrusteanfilterstate.go
Description:
	An implementation of the Procrustean filter as described by Yulin Zhang and
	Dylan A. Shell in their 2020 WAFR paper "Cover Combinatorial Filters and
	their Minimization Problem".
*/

package procrustean

import (
	"errors"
	"fmt"

	oze "github.com/kwesiRutledge/OzayGroupExploration"
)

type ProcrusteanFilterState struct {
	Name   string
	Filter *ProcrusteanFilter
}

/*
ReachedBy
Description:

Assumption:
	Assumes that the state stateIn was checked before running.
	Similarly that the observation sequence was checked.
*/
func (finalState ProcrusteanFilterState) ReachedBy(observationSequence []string, initialState ProcrusteanFilterState) bool {
	// Constants

	// Check Observation Sequence
	return finalState.In(initialState.ReachesWith(observationSequence))
}

/*
ReachesWith
Description:

	An implemenation of V_stateIn(F,s) from Definition 2.
Assumption:
	Assumes that the state stateIn was checked before running.
*/
func (stateIn ProcrusteanFilterState) ReachesWith(observationSequence []string) []ProcrusteanFilterState {
	// Constants

	// Algorithm

	// Iterate through Observarion Sequence to Identify Whether or Not the Final State is Reached.
	var statesReachedAtTime [][]ProcrusteanFilterState
	statesReachedAtTime = append(statesReachedAtTime, []ProcrusteanFilterState{stateIn})
	for tau, observationAtTau := range observationSequence {
		// Find Which transitions exist between the current state
		currentStates := statesReachedAtTime[tau]
		var successorStates []ProcrusteanFilterState
		for _, currentState := range currentStates {
			//and some other state, when given the current observation (observationAtTau)
			var successorsOfCurrentState []ProcrusteanFilterState
			tempPost, _ := Post(currentState, observationAtTau)

			// Append values from tempPost to successorStates
			for _, postState := range tempPost {
				successorsOfCurrentState = postState.AppendIfUniqueTo(successorsOfCurrentState)
				successorStates = postState.AppendIfUniqueTo(successorStates)
			}

		}
		statesReachedAtTime = append(statesReachedAtTime, successorStates)
		// fmt.Println(fmt.Sprintf("Time = %v ; ", tau))
		// fmt.Println(successorStates)
	}
	return statesReachedAtTime[len(observationSequence)]

}

/*
Post
Description:
	Finds the set of states that can follow a given state (or set of states).
	Specifically for ProcrusteanFilter objects.
Usage:
	ancestorStates, err := Post( initialState )
	ancestorStates, err := Post( initialState , actionIn )
*/
func Post(SorSY ...interface{}) ([]ProcrusteanFilterState, error) {
	switch len(SorSY) {
	case 1:
		// Only State Is Given
		stateIn, ok := SorSY[0].(ProcrusteanFilterState)
		if !ok {
			return []ProcrusteanFilterState{}, errors.New("The first input to post is not of type TransitionSystemState.")
		}

		Filter := stateIn.Filter
		Y := Filter.Y

		var nextStates []ProcrusteanFilterState
		var tempPost []ProcrusteanFilterState
		var err error

		for _, y := range Y {
			tempPost, err = Post(stateIn, y)
			if err != nil {
				return []ProcrusteanFilterState{}, err
			}
			for _, postElt := range tempPost {
				nextStates = postElt.AppendIfUniqueTo(nextStates)
			}
		}

		return nextStates, nil

	case 2:
		// State and Action is Given
		stateIn, ok := SorSY[0].(ProcrusteanFilterState)
		if !ok {
			return []ProcrusteanFilterState{}, errors.New("The first input to post is not of type TransitionSystemState.")
		}

		yIn, ok := SorSY[1].(string)
		if !ok {
			return []ProcrusteanFilterState{}, errors.New("The second input to post is not of type string!")
		}

		// Get Transition value
		Filter := stateIn.Filter
		TofS := Filter.tau[stateIn]

		var nextStates []ProcrusteanFilterState

		for nextState, YSubset := range TofS {
			if _, tf := oze.FindStringInSlice(yIn, YSubset); tf {
				nextStates = nextState.AppendIfUniqueTo(nextStates)
			}
		}

		return nextStates, nil
	}

	// Return error
	return []ProcrusteanFilterState{}, errors.New(fmt.Sprintf("Unexpected number of inputs to post (%v).", len(SorSY)))
}

/*
Equals
Description:
	Returns true if the two ProcrusteanFilterState objects have the same name.
*/
func (stateIn ProcrusteanFilterState) Equals(stateTwo ProcrusteanFilterState) bool {
	return stateIn.Name == stateTwo.Name
}

/*
AppendIfUniqueTo
Description:
	Appends the state stateIn to the slice stateSlice only if stateSlice doesn't already contain it.
*/
func (stateIn ProcrusteanFilterState) AppendIfUniqueTo(stateSlice []ProcrusteanFilterState) []ProcrusteanFilterState {
	// Constants

	// Algorithm
	if stateIn.In(stateSlice) {
		return stateSlice
	} else {
		return append(stateSlice, stateIn)
	}

}

/*
Find
Description:
	This algorithm will:
	- if stateIn is in stateSlice, this returns the index where stateIn is found in stateSlice
	- if stateIn is NOT in stateSlice, this returns -1
*/
func (stateIn ProcrusteanFilterState) Find(stateSlice []ProcrusteanFilterState) int {
	// Constants

	// Algorithm
	for stateIndex, tempState := range stateSlice {
		if tempState.Equals(stateIn) {
			return stateIndex
		}
	}

	return -1
}

/*
In
Description:
	This function returns a boolean explaining if a state is in the slice stateSlice or not.
*/
func (stateIn ProcrusteanFilterState) In(stateSlice []ProcrusteanFilterState) bool {
	return stateIn.Find(stateSlice) != -1
}

/*
HasExtension
Description:
	Determines if the ProcrusteanFilterState object has an extension which is ExtensionCandidate type.
*/
func (stateIn ProcrusteanFilterState) HasExtension(extensionIn ExtensionCandidate) bool {
	// Input Checking

	// Constants

	// Algorithm
	statesAtk := []ProcrusteanFilterState{stateIn}
	var statesAtkPlus1 []ProcrusteanFilterState

	for _, observation := range extensionIn.s {
		// Make the set of states at time t+1 a bit larger.
		statesAtkPlus1 = []ProcrusteanFilterState{}

		// Compute Post for each state in stateAtk
		for _, stateAtk := range statesAtk {
			tempPost, _ := Post(stateAtk, observation)

			for _, stateAtkPlus1 := range tempPost {
				statesAtkPlus1 = stateAtkPlus1.AppendIfUniqueTo(statesAtkPlus1)
			}
		}
	}

	return len(statesAtkPlus1) != 0
}

/*
String
Description:
	Returns the name of the state.
*/
func (stateIn ProcrusteanFilterState) String() string {
	return stateIn.Name
}

/*
IsInitial
Description:
	Returns a boolean and describes whether or not the state is an initial state.
*/
func (stateIn ProcrusteanFilterState) IsInitial() bool {
	// Constants
	Filter := stateIn.Filter

	// Algorithm
	return stateIn.In(Filter.V0)
}

/*
IntersectionOfStates
Description:
	Intersects the slices given in the
*/
func IntersectionOfStates(stateSlice1 []ProcrusteanFilterState, otherStateSlices ...[]ProcrusteanFilterState) []ProcrusteanFilterState {
	// Constants
	numOtherSlices := len(otherStateSlices)

	// Algorithm
	var stateSliceIntersection []ProcrusteanFilterState
	for _, tempState := range stateSlice1 {
		tempStateIsInAllOtherSlices := true
		for otherSliceIndex := 0; otherSliceIndex < numOtherSlices; otherSliceIndex++ {
			tempStateIsInAllOtherSlices = tempStateIsInAllOtherSlices && tempState.In(otherStateSlices[otherSliceIndex])
		}
		// Append to stateSliceIntersection
		if tempStateIsInAllOtherSlices {
			stateSliceIntersection = append(stateSliceIntersection, tempState)
		}
	}

	return stateSliceIntersection
}

/*
LanguageWithMaxLength()
Description:
	Creates a set (aka slice) of extensions for the given state.
*/
func (stateIn ProcrusteanFilterState) LanguageWithMaxLength(lengthIn int) ([]ExtensionCandidate, error) {
	// Constants
	pf := stateIn.Filter

	// Input Processing
	if lengthIn <= 0 {
		return []ExtensionCandidate{}, fmt.Errorf("The input length %v is not positive! Please provide a positive length input!", lengthIn)
	}

	// Algorithm
	var extensionsAt [][]ExtensionCandidate
	var extensionCandidatesAt [][]ExtensionCandidate

	extensionsAt = append(extensionsAt, []ExtensionCandidate{})
	extensionCandidatesAt = append(extensionCandidatesAt, []ExtensionCandidate{})

	var lengthOneSequences []ExtensionCandidate
	for observationIndex := 0; observationIndex < len(pf.Y); observationIndex++ {
		lengthOneSequences = append(lengthOneSequences,
			ExtensionCandidate{s: []string{pf.Y[observationIndex]}, Filter: pf},
		)
	}
	extensionCandidatesAt = append(extensionCandidatesAt, lengthOneSequences)

	s0 := stateIn

	// Use loop to test new candidates and expand the set of candidates
	for T := 1; T <= lengthIn; T++ {
		// Keep track of all extension candidates which are confirmed.
		var confirmedExtensionsAtT []ExtensionCandidate
		for _, extensionCandidate := range extensionCandidatesAt[T] {
			if extensionCandidate.IsExtensionOf(s0) {
				// If this is a true extension of s0, then add it to the list of true
				confirmedExtensionsAtT = extensionCandidate.AppendIfUniqueTo(confirmedExtensionsAtT)
			}
		}
		extensionsAt = append(extensionsAt, confirmedExtensionsAtT)

		// Extend the confirmed extensions
		var nextCandidates []ExtensionCandidate
		for _, tempExtension := range confirmedExtensionsAtT {
			nextCandidates = append(nextCandidates, tempExtension.ExtendByOne()...)
		}
		extensionCandidatesAt = append(extensionCandidatesAt, nextCandidates)
	}

	return extensionsAt[len(extensionsAt)-1], nil

}
