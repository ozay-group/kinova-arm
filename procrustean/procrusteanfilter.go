/*
procrusteanfilter.go
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

type ProcrusteanFilter struct {
	V   []ProcrusteanFilterState                                       // State Space
	V0  []ProcrusteanFilterState                                       // Initial States
	Y   []string                                                       // Observations
	tau map[ProcrusteanFilterState]map[ProcrusteanFilterState][]string // Transition Mapping
	C   []string                                                       // Set of Outputs
	c   map[ProcrusteanFilterState][]string                            // Output Mapping
}

type ProcrusteanFilterState struct {
	Name   string
	Filter *ProcrusteanFilter
}

// type ProcrusteanFilterObservation struct {
// 	Name   string
// 	Filter *ProcrusteanFilter
// }

func GetProcrusteanFilter(stateNames []string, initialStateNames []string, observationStrings []string, transitionMap map[string]map[string][]string, outputStrings []string, outputMapIn map[string][]string) (ProcrusteanFilter, error) {

	// Create Base ProcrusteanFilter
	pf := ProcrusteanFilter{
		Y: observationStrings,
		C: outputStrings,
	}

	// Create List of States
	var V []ProcrusteanFilterState //List Of States
	for _, stateName := range stateNames {
		V = append(V, ProcrusteanFilterState{Name: stateName, Filter: &pf})
	}
	pf.V = V

	// Create List of Initial States
	var V0 []ProcrusteanFilterState
	for _, stateName := range initialStateNames {
		V0 = append(V0, ProcrusteanFilterState{Name: stateName, Filter: &pf})
	}
	pf.V0 = V0

	// Create Transition Map
	Transition := make(map[ProcrusteanFilterState]map[ProcrusteanFilterState][]string)
	for siName, perStateMap := range transitionMap {
		si := ProcrusteanFilterState{Name: siName, Filter: &pf}
		tempNextStateMap := make(map[ProcrusteanFilterState][]string)
		for siPlus1Name, observationArray := range perStateMap {
			siPlus1 := ProcrusteanFilterState{Name: siPlus1Name, Filter: &pf}
			tempNextStateMap[siPlus1] = observationArray
		}
		Transition[si] = tempNextStateMap
	}
	pf.tau = Transition

	// Create Output map
	fullOutputMap := make(map[ProcrusteanFilterState][]string)
	for stateValue, associatedOutputs := range outputMapIn {
		tempState := ProcrusteanFilterState{Name: stateValue, Filter: &pf}
		fullOutputMap[tempState] = associatedOutputs
	}
	pf.c = fullOutputMap

	return pf, nil

}

func (finalState ProcrusteanFilterState) ReachedBy(observationSequence []string, initialState ProcrusteanFilterState) (bool, error) {
	// Get Filter
	pf := finalState.Filter

	n := len(observationSequence)

	// Check Observation Sequence
	for _, tempObservation := range observationSequence {
		if _, isInY := oze.FindStringInSlice(tempObservation, pf.Y); !isInY {
			return false, fmt.Errorf("The observation \"%v\" was not found in the parent Filter. Please check filter definition.", tempObservation)
		}
	}

	// Iterate through Observarion Sequence to Identify Whether or Not the Final State is Reached.
	var statesReachedAtTime [][]ProcrusteanFilterState
	statesReachedAtTime = append(statesReachedAtTime, []ProcrusteanFilterState{initialState})
	for tau, observationAtTau := range observationSequence {
		// Find Which transitions exist between the current state
		currentStates := statesReachedAtTime[tau]
		var successorStates []ProcrusteanFilterState
		for _, currentState := range currentStates {
			//and some other state, when given the current observation (observationAtTau)
			var successorsOfCurrentState []ProcrusteanFilterState
			for _, candidateNextState := range pf.V {
				if _, containsObservationAtTau := oze.FindStringInSlice(observationAtTau, pf.tau[currentState][candidateNextState]); containsObservationAtTau {
					successorsOfCurrentState = append(successorsOfCurrentState, candidateNextState)
					successorStates = append(successorStates, candidateNextState)
				}
			}
		}
		statesReachedAtTime = append(statesReachedAtTime, successorStates)
		// fmt.Println(fmt.Sprintf("Time = %v ; ", tau))
		// fmt.Println(successorStates)
	}

	// Check to see if the states reached at time len(observationSequence) contains the target.
	for _, reachedState := range statesReachedAtTime[n] {
		if reachedState.Name == finalState.Name {
			return true, nil
		}
	}

	return false, nil
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
