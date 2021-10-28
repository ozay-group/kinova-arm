/*
zipperconstraint.go
Description:
	An implementation of the Zipper Constraint Candidate and its functions as described by Yulin Zhang and
	Dylan A. Shell in their 2020 WAFR paper "Cover Combinatorial Filters and
	their Minimization Problem".
*/
package procrustean

import oze "github.com/kwesiRutledge/OzayGroupExploration"

// Type Definitions
// ================

type ZipperConstraintCandidate struct {
	U      []ProcrusteanFilterState
	W      []ProcrusteanFilterState
	y      string             // observation
	Filter *ProcrusteanFilter // Pointer to the Filter with states making U and W
}

// Methods
// =======
func ToZipperConstraintCandidate(U, W []ProcrusteanFilterState, y string) ZipperConstraintCandidate {
	return ZipperConstraintCandidate{
		U:      U,
		W:      W,
		y:      y,
		Filter: U[0].Filter,
	}
}

func (zcc ZipperConstraintCandidate) IsZipperConstraint() bool {
	// Constants
	Filter := zcc.Filter
	G := Filter.ToCompatibilityGraph()

	// Algorithm
	if !IsMutuallyCompatibleSet(zcc.U) || !IsMutuallyCompatibleSet(zcc.W) {
		// One of the sets is not Mutually Compatible
		return false
	}

	// Verify that the observation y leads to the transition from a u in U to a w in W.
	var tempW []ProcrusteanFilterState
	for _, w := range G.V {
		// Find a u that might satisfy the condition.
		for _, u := range zcc.U {
			if _, tf := oze.FindStringInSlice(zcc.y, Filter.tau[u][w]); tf {
				tempW = append(tempW, w)
			}
		}
	}

	// Is tempW the same as W? IF so return true.
	tf, _ := SliceEquals(tempW, zcc.W)
	return tf
}
