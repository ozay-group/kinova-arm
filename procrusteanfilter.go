/*
procrusteanfilter.go
Description:
	An implementation of the Procrustean filter as described by Yulin Zhang and
	Dylan A. Shell in their 2020 WAFR paper "Cover Combinatorial Filters and
	their Minimization Problem".
*/

package OzayGroupExploration

import "fmt"

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

type ProcrusteanFilterObservation struct {
	Name   string
	Filter *ProcrusteanFilter
}

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

/*
FindStringInSlice
Description:
	Finds if the string stringIn is in the slice stringSliceIn.
*/
func FindStringInSlice(stringIn string, stringSliceIn []string) (int, bool) {
	// Initialize search parameters
	stringIndex := -1

	// Search
	for tempIndex, tempString := range stringSliceIn {
		if tempString == stringIn {
			stringIndex = tempIndex
		}
	}

	// Return result
	return stringIndex, stringIndex >= 0
}

func (finalState ProcrusteanFilterState) ReachedBy(observationSequence []string, initialState ProcrusteanFilterState) (bool, error) {
	// Get Filter
	pf := finalState.Filter

	n := len(observationSequence)

	// Check Observation Sequence
	for _, tempObservation := range observationSequence {
		if _, isInY := FindStringInSlice(tempObservation, pf.Y); !isInY {
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
				if _, containsObservationAtTau := FindStringInSlice(observationAtTau, pf.tau[currentState][candidateNextState]); containsObservationAtTau {
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
