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
	Act        []string
	Transition map[TransitionSystemState]map[string][]TransitionSystemState
	I          []TransitionSystemState // Set of Initial States
	AP         []AtomicProposition
	L          map[TransitionSystemState][]AtomicProposition
}

type TransitionSystemState struct {
	Name   string
	System *TransitionSystem
}

func (s1 TransitionSystemState) Equals(s2 TransitionSystemState) bool {
	return s1.Name == s2.Name
}

// /*
// 	GetState
// 	Description:
// 		Retrieves a StateOfTransitionSystem object from the transition system
// 		ts.
// */
// func (ts TransitionSystem) GetState(stateNameIn string) (TransitionSystemState, error) {

// 	// Checking Inputs
// 	if (stateNameIn < 0) || (state_in > len(ts.S)) {
// 		return TransitionSystemState{Name: ""}, errors.New(fmt.Sprintf("The input to GetState (%v) is not a state from the target transition system.", state_in))
// 	}

// 	// Return Simple State
// 	return TransitionSystemState{Name: state_in, System: &ts}, nil

// }

type AtomicProposition struct {
	Name string
}

/*
Equals
Description:
	Compares the atomic propositions by their names.
*/
func (ap1 AtomicProposition) Equals(ap2 AtomicProposition) bool {

	return ap1.Name == ap2.Name

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
func GetTransitionSystem(stateNames []string, actionNames []string, transitionMap map[string]map[string][]string, initialStateList []string, atomicPropositionsList []string, labelMap map[string][]string) (TransitionSystem, error) {
	// Constants

	// Algorithm
	ts := TransitionSystem{
		Act: actionNames,
		AP:  StringSliceToAPs(atomicPropositionsList),
	}

	// Create List of States
	var S []TransitionSystemState //List Of States
	for _, stateName := range stateNames {
		S = append(S, TransitionSystemState{Name: stateName, System: &ts})
	}
	ts.S = S

	// Create List of Initial States
	var I []TransitionSystemState
	for _, stateName := range initialStateList {
		I = append(I, TransitionSystemState{Name: stateName, System: &ts})
	}
	ts.I = I

	// // Create List of Atomic Propositions
	// ts.AP = StringSliceToAPs(atomicPropositionsList)

	// Create Transition Map
	Transition := make(map[TransitionSystemState]map[string][]TransitionSystemState)
	for siName, perStateMap := range transitionMap {
		si := TransitionSystemState{Name: siName, System: &ts}
		tempActionMap := make(map[string][]TransitionSystemState)
		for actionName, stateArray := range perStateMap {
			var tempStates []TransitionSystemState
			for _, siPlus1Name := range stateArray {
				tempStates = append(tempStates, TransitionSystemState{Name: siPlus1Name, System: &ts})
			}
			tempActionMap[actionName] = tempStates
		}
		Transition[si] = tempActionMap
	}
	ts.Transition = Transition

	// Create Label Values
	fullLabelMap := make(map[TransitionSystemState][]AtomicProposition)
	for stateValue, associatedAPs := range labelMap {
		tempState := TransitionSystemState{Name: stateValue, System: &ts}
		fullLabelMap[tempState] = StringSliceToAPs(associatedAPs)
	}
	ts.L = fullLabelMap

	return ts, nil
}

/*
In
Description:
	Determines if the state is in a given slice of TransitionSystemState objects.
Usage:
	tf := s1.In( sliceIn )
*/
func (stateIn TransitionSystemState) In(stateSliceIn []TransitionSystemState) bool {

	for _, tempState := range stateSliceIn {
		if tempState.Equals(stateIn) {
			return true //If there is a match, then return true.
		}
	}

	//If there is no match in the slice,
	//then return false
	return false

}

/*
Satisfies
Description:
	The state of the transition system satisfies the given formula.
*/

func (stateIn TransitionSystemState) Satisfies(formula interface{}) (bool, error) {

	// Input Processing
	if stateIn.System == nil {
		return false, errors.New("The system pointer is not defined for the input state.")
	}

	// Algorithm
	var tf = false
	var err error = nil

	if singleAP, ok := formula.(AtomicProposition); ok {
		tf, err = singleAP.SatisfactionHelper(stateIn)
	}

	return tf, err
}

func (ap AtomicProposition) SatisfactionHelper(stateIn TransitionSystemState) (bool, error) {
	// Constants

	SystemPointer := stateIn.System
	LOfState := SystemPointer.L[stateIn]

	// Find If ap is in LOfState
	tf := false
	for _, tempAP := range LOfState {
		tf = tf || tempAP.Equals(ap)
	}

	return tf, nil

}

/*
AppendIfUnique
Description:
	Appends to the input slice sliceIn if and only if the new state
	is actually a unique state.
*/
func AppendIfUnique(sliceIn []TransitionSystemState, stateIn TransitionSystemState) []TransitionSystemState {
	// Check to see if the State is equal to any of the ones in the list.
	for _, stateFromSlice := range sliceIn {
		if stateFromSlice.Equals(stateIn) {
			return sliceIn
		}
	}

	// If none of the states in sliceIn are equal to stateIn, then return the appended version of sliceIn.
	return append(sliceIn, stateIn)

}

/*
Post
Description:
	Finds the set of states that can follow a given state (or set of states).
Usage:

*/

func Post(SorSA ...interface{}) ([]TransitionSystemState, error) {
	switch len(SorSA) {
	case 1:
		// Only State Is Given
		stateIn, ok := SorSA[0].(TransitionSystemState)
		if !ok {
			return []TransitionSystemState{}, errors.New("The first input to post is not of type TransitionSystemState.")
		}

		System := stateIn.System
		allActions := System.Act

		var nextStates []TransitionSystemState
		var tempPost []TransitionSystemState
		var err error
		for _, action := range allActions {
			tempPost, err = Post(stateIn, action)
			if err != nil {
				return []TransitionSystemState{}, err
			}
			for _, postElt := range tempPost {
				nextStates = AppendIfUnique(nextStates, postElt)
			}
		}

		return nextStates, nil

	case 2:
		// State and Action is Given
		stateIn, ok := SorSA[0].(TransitionSystemState)
		if !ok {
			return []TransitionSystemState{}, errors.New("The first input to post is not of type TransitionSystemState.")
		}

		actionIn, ok := SorSA[1].(string)
		if !ok {
			return []TransitionSystemState{}, errors.New("The second input to post is not of type string!")
		}

		// Get Transition value
		System := stateIn.System
		tValues := System.Transition[stateIn][actionIn]
		var nextStates []TransitionSystemState
		for _, nextState := range tValues {
			nextStates = AppendIfUnique(nextStates, nextState)
		}

		return nextStates, nil
	}

	// Return error
	return []TransitionSystemState{}, errors.New(fmt.Sprintf("Unexpected number of inputs to post (%v).", len(SorSA)))
}

/*
Pre
Description:
	Finds the set of states that can precede a given state (or set of states).
Usage:

*/

func Pre(SorSA ...interface{}) ([]TransitionSystemState, error) {
	switch len(SorSA) {
	case 1:
		// Only State Is Given
		stateIn, ok := SorSA[0].(TransitionSystemState)
		if !ok {
			return []TransitionSystemState{}, errors.New("The first input to post is not of type TransitionSystemState.")
		}

		System := stateIn.System
		allActions := System.Act

		var predecessors []TransitionSystemState
		var tempPre []TransitionSystemState
		var err error
		for _, action := range allActions {
			tempPre, err = Pre(stateIn, action)
			if err != nil {
				return []TransitionSystemState{}, err
			}
			for _, preElt := range tempPre {
				predecessors = AppendIfUnique(predecessors, preElt)
			}
		}

		return predecessors, nil

	case 2:
		// State and Action is Given
		stateIn, ok := SorSA[0].(TransitionSystemState)
		if !ok {
			return []TransitionSystemState{}, errors.New("The first input to post is not of type TransitionSystemState.")
		}

		actionIn, ok := SorSA[1].(string)
		if !ok {
			return []TransitionSystemState{}, errors.New("The second input to post is not of type string!")
		}

		// Get Transition value
		System := stateIn.System
		var matchingPredecessors []TransitionSystemState
		for predecessor, actionMap := range System.Transition {
			if stateIn.In(actionMap[actionIn]) {
				// If the target state is in the result of (predecessor,actionIn) -> stateIn,
				// then save the value of stateIn
				matchingPredecessors = append(matchingPredecessors, predecessor)
			}
		}

		return matchingPredecessors, nil
	}

	// Return error
	return []TransitionSystemState{}, errors.New(fmt.Sprintf("Unexpected number of inputs to post (%v).", len(SorSA)))
}
