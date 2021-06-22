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
	S          []int
	Act        []int
	Transition map[int]int
	I          []int // Set of Initial States
	AP         []string
	L          map[int][]string
}

type StateOfTransitionSystem struct {
	Value  int
	System TransitionSystem
}

/*
	GetState
	Description:
		Retrieves a StateOfTransitionSystem object from the transition system
		ts.
*/
func (ts TransitionSystem) GetState(state_in int) (StateOfTransitionSystem, error) {

	// Checking Inputs
	if (state_in < 0) || (state_in > len(ts.S)) {
		return StateOfTransitionSystem{Value: -1}, errors.New(fmt.Sprintf("The input to GetState (%v) is not a state from the target transition system.", state_in))
	}

	// Return Simple State
	return StateOfTransitionSystem{Value: state_in, System: ts}, nil

}

type AtomicProposition struct {
}

/*

 */
