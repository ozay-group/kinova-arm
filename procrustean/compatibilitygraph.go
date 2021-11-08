/*
compatibilitygraph.go
Description:
	An implementation of the Compatibility Graph and its functions as described by Yulin Zhang and
	Dylan A. Shell in their 2020 WAFR paper "Cover Combinatorial Filters and
	their Minimization Problem".
*/
package procrustean

// Type Definitions
// ================

type CompatibilityGraph struct {
	V      []ProcrusteanFilterState
	E      [][2]ProcrusteanFilterState
	Filter *ProcrusteanFilter
}

// Functions
// =========

/*
ContainsEdge
Description:
	Determines if an edge is in the edge
*/
func (cg *CompatibilityGraph) ContainsEdge(candidateEdge [2]ProcrusteanFilterState) bool {
	// Constants

	// Algorithm
	for _, edge := range cg.E {
		if edge[0].Equals(candidateEdge[0]) && edge[1].Equals(candidateEdge[1]) {
			return true
		}
	}

	// If no matches were found, then return false.
	return false
}

/*
AddEdge
Description:
	Adds an edge to the CompatibilityGraph if and only the edge is not already found there.
*/
func (cg *CompatibilityGraph) AddEdge(candidateEdge [2]ProcrusteanFilterState) {
	// Constants

	// Algorithm
	if !cg.ContainsEdge(candidateEdge) {
		cg.E = append(cg.E, candidateEdge)
	}
}

func (cg *CompatibilityGraph) AllVerticesConnectedTo(v ProcrusteanFilterState) []ProcrusteanFilterState {
	// Constants

	// Input Processing
	if !v.In(cg.V) {
		return []ProcrusteanFilterState{}
	}

	// Algorithm
	R1 := append([]ProcrusteanFilterState{v}, cg.AllVerticesAdjacentTo(v)...)
	R0 := []ProcrusteanFilterState{}

	R1equalsR0, _ := SliceEquals(R1, R0)
	for !R1equalsR0 {
		// Make R0 equal R1
		R0 = R1

		// Make R1 be equal to the expanded set
		for _, v := range R1 {
			R1 = UnionOfStates(R1, cg.AllVerticesAdjacentTo(v))
		}

		// Determine if R1 equals R0
		R1equalsR0, _ = SliceSubset(R1, R0)
	}

	return R0
}

/*
AllVerticesAdjacentTo
Description:
	Collects all vertices ADJACENT to the input v in the graph.
*/
func (cg *CompatibilityGraph) AllVerticesAdjacentTo(v ProcrusteanFilterState) []ProcrusteanFilterState {
	// Constants

	// Input Processing
	if !v.In(cg.V) {
		return []ProcrusteanFilterState{}
	}

	// Algorithm
	var adjacentTov []ProcrusteanFilterState
	for _, tempEdge := range cg.E {
		// if v is the source of the edge
		if tempEdge[0].Equals(v) {
			adjacentTov = tempEdge[1].AppendIfUniqueTo(adjacentTov)
		}
		// if v is the target of the edge
		if tempEdge[1].Equals(v) {
			adjacentTov = tempEdge[0].AppendIfUniqueTo(adjacentTov)
		}
	}

	return adjacentTov
}

/*
GetSetsOfCompatibleStates
Description:

*/
func (cg *CompatibilityGraph) GetSetsOfCompatibleStates() [][]ProcrusteanFilterState {
	// Constants

	// Algorithm
	var SetsOfCompatibleStates [][]ProcrusteanFilterState

	for _, v := range cg.V {
		// Determine if v is already in SetsOfCompatibleStates
		var vIsAlreadyIncluded = false
		for _, tempSet := range SetsOfCompatibleStates {
			if v.In(tempSet) {
				vIsAlreadyIncluded = true
			}
		}

		// If v is NOT already included, then get all states connected to v
		if !vIsAlreadyIncluded {
			connectedTov := cg.AllVerticesConnectedTo(v)
			SetsOfCompatibleStates = append(SetsOfCompatibleStates, connectedTov)
		}
	}

	return SetsOfCompatibleStates
}

/*
GetAllZipperConstraints
Description:
	Searches through the compatibility graph and returns arll possible zipper constraints.
*/
func (cg *CompatibilityGraph) GetAllZipperConstraints() []ZipperConstraint {
	// Constants
	MaximalSetsOfCompatibleStates := cg.GetSetsOfCompatibleStates()
	F := cg.Filter

	// Algorithm

	var allZipperConstraints []ZipperConstraint

	var allSubsetsOfMaximalCompatibleStates [][]ProcrusteanFilterState
	for _, maximalSet1 := range MaximalSetsOfCompatibleStates {
		powersetOfMaximalSet1 := Powerset(maximalSet1)
		allSubsetsOfMaximalCompatibleStates = append(allSubsetsOfMaximalCompatibleStates, powersetOfMaximalSet1...)
	}

	// Now get combinations of subsets
	for _, subset1 := range allSubsetsOfMaximalCompatibleStates {

		if len(subset1) == 0 {
			continue
		}

		for _, subset2 := range allSubsetsOfMaximalCompatibleStates {

			if len(subset2) == 0 {
				continue
			}

			// Iterate through Y from the procrustean filter
			for _, y := range F.Y {
				// Debug
				// fmt.Print(subset1)
				// fmt.Print(subset2)
				// fmt.Println(y)

				// Check to see if these subsets + y make a ZipperConstraint
				zc, err := GetZipperConstraint(subset1, subset2, y)
				if err == nil {
					allZipperConstraints = append(allZipperConstraints, zc)
				}
			}
		}
	}

	// rreturn the zipper constraints
	return allZipperConstraints
}
