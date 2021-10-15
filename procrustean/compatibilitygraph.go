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
