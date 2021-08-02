/*
transitionsystem.go
Description:
 	Basic implementation of a Transition System.
*/
package OzayGroupExploration

import (
	"errors"
	"fmt"
)

type TransitionSystem struct {
	S          []TransitionSystemState
	Act        []int
	Transition map[int][][]int
	I          []int // Set of Initial States
	AP         []AtomicProposition
	L          map[int][]AtomicProposition
}

type TransitionSystemState struct {
	Value  int
	System *TransitionSystem
}

/*
	GetState
	Description:
		Retrieves a StateOfTransitionSystem object from the transition system
		ts.
*/
func (ts TransitionSystem) GetState(state_in int) (TransitionSystemState, error) {

	// Checking Inputs
	if (state_in < 0) || (state_in > len(ts.S)) {
		return TransitionSystemState{Value: -1}, errors.New(fmt.Sprintf("The input to GetState (%v) is not a state from the target transition system.", state_in))
	}

	// Return Simple State
	return TransitionSystemState{Value: state_in, System: &ts}, nil

}

type AtomicProposition struct {
	Name string
}

/*
StringSliceToAPs
Description:
	Transforms a slice of int variables into a list of AtomicPropositions
*/
func StringSliceToAPs(stringSlice []string) []AtomicProposition {
	var APList []AtomicProposition
	for _, apName := range stringSlice {
		APList = append(APList, AtomicProposition{Name: apName})
	}

	return APList
}

/*
GetTransitionSystem
Description:

*/
func GetTransitionSystem(stateIndicies []int, actionIndicies []int, transitionMap map[int][][]int, initialStateList []int, atomicPropositionsList []string, labelMap map[int][]string) (TransitionSystem, error) {
	// Constants

	// Algorithm
	ts := TransitionSystem{
		Act:        actionIndicies,
		Transition: transitionMap,
		I:          initialStateList,
	}

	// Create List of States
	var S []TransitionSystemState //List Of States
	for _, stateValue := range stateIndicies {
		S = append(S, TransitionSystemState{Value: stateValue, System: &ts})
	}
	ts.S = S

	// Create List of Atomic Propositions
	ts.AP = StringSliceToAPs(atomicPropositionsList)

	// Create Label Values
	fullLabelMap := make(map[TransitionSystemState][]AtomicProposition)
	for stateValue, associatedAPs := range labelMap {
		tempState := TransitionSystemState{Value: stateValue, System: &ts}
		fullLabelMap[tempState] = StringSliceToAPs(associatedAPs)
	}

	return ts, nil
}
