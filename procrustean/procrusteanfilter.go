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

/*
IsDeterministic
Description:
	According to the paper, the p-filter is deterministic or state-determined if:
	- |V0|=1, and
	- for every v1, v2,v3 in V with v2 =/= v3, tau(v1,v2) \cap tau(v1,v3) = \emptyset
*/
func (pf ProcrusteanFilter) IsDeterministic() bool {
	// Constants

	// Algorithm

	// |V0|=1 should be satisfied
	if len(pf.V0) != 1 {
		return false
	}

	// for every v1, v2,v3 in V with v2 =/= v3, tau(v1,v2) \cap tau(v1,v3) = \emptyset
	for _, v1 := range pf.V {
		for _, v2 := range pf.V {
			for _, v3 := range pf.V {
				// Skip if v2 == v3
				if v2.Equals(v3) {
					continue
				}

				// Otherwise, compute tau(v1,v2) and tau(v1,v3), then intersect them
				tau12 := pf.tau[v1][v2]
				tau13 := pf.tau[v1][v3]

				tauIntersect := IntersectionOfStringSlices(tau12, tau13)
				// If the intersection is NOT empty, then return false.
				if len(tauIntersect) != 0 {
					return false
				}

			}
		}
	}

	return true
}

/*
ToCompatibilityGraph()
Description:

*/
func (pf ProcrusteanFilter) ToCompatibilityGraph() CompatibilityGraph {
	// Constants
	numStates := len(pf.V)

	// Algorihtm
	cgOut := CompatibilityGraph{
		Filter: &pf,
	}

	cgOut.V = pf.V

	// Build Edges
	for sourceIndex, tempSource := range pf.V {
		for targetIndex := sourceIndex + 1; targetIndex < numStates; targetIndex++ {

			// Create sink state
			tempTarget := pf.V[targetIndex]

			// IF source and sink are the same, then skip this iteration.
			if tempSource.Equals(tempTarget) {
				continue
			}

			// Otherwise, make an edge if and only if the two states are compatible
			if tempSource.IsCompatibleWith(tempTarget) {
				cgOut.AddEdge([2]ProcrusteanFilterState{tempSource, tempTarget})
			}

		}
	}

	// Return final result
	return cgOut
}
