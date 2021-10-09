package procrustean

/*
examples_test.go
Description:
	Tests the functions defined in the examples.go file.
*/

import (
	"testing"
)

/*
TestExamples_GetPFilter1_1
Description:
	Tests the formalized function GetPFilter1 and identifies if the right number of states are in it.
*/
func TestExamples_GetPFilter1_1(t *testing.T) {
	// Create Basic PF
	pf1 := GetPFilter1()

	// Algorithm
	if len(pf1.V) != 10 {
		t.Errorf("Expected for 10 states to be in Procrustean filter but received %v.", len(pf1.V))
	}

	if len(pf1.C) != 5 {
		t.Errorf("Expected for 5 observations to be in Procrustean filter but received %v.", len(pf1.C))
	}
}

/*
TestExamples_GetPFilter2_1
Description:
	Tests the formalized function GetPFilter2 and identifies if the right number of states and observations are in it.
*/
func TestExamples_GetPFilter2_1(t *testing.T) {
	// Create Basic PF
	pf1 := GetPFilter2()

	// Algorithm
	if len(pf1.V) != 4 {
		t.Errorf("Expected for 4 states to be in Procrustean Filter but received %v.", len(pf1.V))
	}

	if len(pf1.C) != 3 {
		t.Errorf("Expected for 3 observations to be in Procrustean Filter but received %v.", len(pf1.C))
	}

}
