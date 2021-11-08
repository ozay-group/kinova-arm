package procrustean

/*
compatibilitygraph_test.go
Description:
	Tests the functions related to the CompatibilityGraph type in this toolbox.
*/

import (
	"testing"
)

/*
TestCompatibilityGraph_GetSetsOfCompatibleStates1
Description:
	This function tests the GetSetsOfCompatibleStates() and verifies that it creates the proper number of states in the induced filter for Filter 5.
*/
func TestCompatibilityGraph_GetSetsOfCompatibleStates1(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()

	cg0 := pf0.ToCompatibilityGraph()

	// Algorithm
	cs0 := cg0.GetSetsOfCompatibleStates()

	if len(cs0) != 6 {
		t.Errorf("The number of sets of compatible sets according to cg0 is %v, but expected 6.", len(cs0))
	}

}

/*
TestCompatibilityGraph_GetSetsOfCompatibleStates2
Description:
	This function tests the GetSetsOfCompatibleStates() and verifies that it creates the proper first subset in the induced filter for Filter 5.
*/
func TestCompatibilityGraph_GetSetsOfCompatibleStates2(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()

	cg0 := pf0.ToCompatibilityGraph()

	// Algorithm
	cs0 := cg0.GetSetsOfCompatibleStates()

	if len(cs0[0]) != 1 {
		t.Errorf("The first subset in cs0 should have one element, but %v were found.", len(cs0[0]))
	}

	if !cs0[0][0].Equals(pf0.V0[0]) {
		t.Errorf("The first set in cs0 should contain only the initial state, but it does not!")
	}

}

/*
TestCompatibilityGraph_GetAllZipperConstraints1
Description:

*/
func TestCompatibilityGraph_GetAllZipperConstraints1(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()
	cg0 := pf0.ToCompatibilityGraph()

	// Algorithm
	zcSlice := cg0.GetAllZipperConstraints()

	if len(zcSlice) == 0 {
		t.Errorf("For some reason the algorithm has not found any Zipper Constraints for pf0!")
	}

	t.Errorf("There are %v zipper constraints!", len(zcSlice))

	for zcIndex, zc := range zcSlice {
		t.Errorf("Zipper Constraint %v: %v", zcIndex, zc)
	}
}
