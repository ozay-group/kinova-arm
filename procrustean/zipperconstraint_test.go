package procrustean

/*
zipperconstraint_test.go
Description:
	Tests the functions related to the ZipperConstraint type in this toolbox.
*/

import (
	"testing"
)

/*
TestZipperConstraint_Check1
Description:
	This function tests the Check() method for a ZipperConstraint.
	Creates a ZipperConstraint which should be valid.
*/
func TestZipperConstraint_Check1(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()

	set1 := pf0.V[1:3]
	set2 := pf0.V[5:7]

	y := "a"

	candidateZC := ZipperConstraint{
		U:      set1,
		W:      set2,
		y:      y,
		Filter: &pf0,
	}

	// Algorithm
	err := candidateZC.Check()

	if err != nil {
		t.Errorf("The candidate candidateZC is known to be correct, but Check() produces an error!")
	}

}

/*
TestZipperConstraint_Check2
Description:
	This function tests the Check() method for a ZipperConstraint.
	Provides a zipper constraint which should be incorrect.
*/
func TestZipperConstraint_Check2(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()

	set1 := pf0.V[3:5]
	set2 := pf0.V[5:7]

	y := "b"

	candidateZC := ZipperConstraint{
		U:      set1,
		W:      set2,
		y:      y,
		Filter: &pf0,
	}

	// Algorithm
	err := candidateZC.Check()

	if err == nil {
		t.Errorf("The candidate candidateZC is known to be incorrect, but Check() does not produce an error!")
	}

}

/*
TestZipperConstraint_GetZipperConstraint1
Description:
	This function tests the Equals() function between two different filter states.
*/
func TestZipperConstraint_GetZipperConstraint1(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()

	set1 := pf0.V[1:3]
	set2 := pf0.V[5:7]

	y := "a"

	// Algorithm
	_, err := GetZipperConstraint(set1, set2, y)

	if err != nil {
		t.Errorf("The function GetZipperConstraint() produced an error when creating a zipper constraint which is known to be good.")
	}
}
