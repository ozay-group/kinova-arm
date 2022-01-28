/*
zipperconstraint.go
Description:
	An implementation of the Zipper Constraint Candidate and its functions as described by Yulin Zhang and
	Dylan A. Shell in their 2020 WAFR paper "Cover Combinatorial Filters and
	their Minimization Problem".
*/
package procrustean

import (
	"fmt"

	oze "github.com/kwesiRutledge/OzayGroupExploration"
)

// Type Definitions
// ================

type ZipperConstraint struct {
	U      []ProcrusteanFilterState
	W      []ProcrusteanFilterState
	y      string             // observation
	Filter *ProcrusteanFilter // Pointer to the Filter with states making U and W
}

// Methods
// =======
func GetZipperConstraint(U, W []ProcrusteanFilterState, y string) (ZipperConstraint, error) {

	candidateZC := ZipperConstraint{
		U:      U,
		W:      W,
		y:      y,
		Filter: U[0].Filter,
	}

	return candidateZC, candidateZC.Check()
}

func (zc ZipperConstraint) Check() error {
	// Constants
	Filter := zc.Filter
	G := Filter.ToCompatibilityGraph()

	// Algorithm
	if !IsMutuallyCompatibleSet(zc.U) || !IsMutuallyCompatibleSet(zc.W) {
		// One of the sets is not Mutually Compatible
		return fmt.Errorf("One of the sets in ZC is not a mutually compatible set!")
	}

	// Verify that the observation y leads to the transition from a u in U to a w in W.
	var tempW []ProcrusteanFilterState
	for _, w := range G.V {
		// Find a u that might satisfy the condition.
		for _, u := range zc.U {
			if _, tf := oze.FindStringInSlice(zc.y, Filter.tau[u][w]); tf {
				tempW = append(tempW, w)
			}
		}
	}

	// Is tempW the same as W? IF so return true.
	tf, _ := SliceEquals(tempW, zc.W)
	if !tf {
		return fmt.Errorf("The set of states that are reached by u with observation y is not the same as W!")
	}

	return nil
}
