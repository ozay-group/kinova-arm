/*
procrusteanfilter.go
Description:
	An implementation of the Procrustean filter as described by Yulin Zhang and
	Dylan A. Shell in their 2020 WAFR paper "Cover Combinatorial Filters and
	their Minimization Problem".
*/

package procrustean

type ProcrusteanFilter struct {
	V   []ProcrusteanFilterState                                       // State Space
	V0  []ProcrusteanFilterState                                       // Initial States
	Y   []string                                                       // Observations
	tau map[ProcrusteanFilterState]map[ProcrusteanFilterState][]string // Transition Mapping
	C   []string                                                       // Set of Outputs
	c   map[ProcrusteanFilterState][]string                            // Output Mapping
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
