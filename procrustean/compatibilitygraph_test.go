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

	if len(cs0) != 5 {
		t.Errorf("The number of sets of compatible sets according to cg0 is %v, but expected 5.", len(cs0))
	}

}
