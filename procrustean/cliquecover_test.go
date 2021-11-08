package procrustean

/*
cliquecover_test.go
Description:
	Tests the functions related to the CliqueCover type in this toolbox.
*/

import (
	"testing"
)

/*
TestCliqueCover_ToInducedFilter1
Description:
	This function tests the ToInducedFilter1() and verifies that it creates the proper number of states in the induced filter for Filter 5.
*/
func TestCliqueCover_ToInducedFilter1(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()

	cc, tf := CreateInducedCliqueCover(pf0, pf0)

	// Algorithm
	if !tf {
		t.Errorf("Filed to create induced clique cover!")
	}

	pf0_prime, err := cc.ToInducedFilter()

	if err != nil {
		t.Errorf("There was an error creating induced filter!")
	}

	if len(pf0_prime.V) != len(cc.K) {
		t.Errorf("The number of states in F' is %v, but there are %v elements in induced clique cover.", len(pf0_prime.V), len(cc.K))
	}

}

/*
TestCliqueCover_ToInducedFilter2
Description:
	This function tests the ToInducedFilter() and verifies that it creates the proper number of initial states in the induced filter for Filter 5.
*/
func TestCliqueCover_ToInducedFilter2(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()

	cc, tf := CreateInducedCliqueCover(pf0, pf0)

	// Algorithm
	if !tf {
		t.Errorf("Filed to create induced clique cover!")
	}

	pf0_prime, err := cc.ToInducedFilter()

	if err != nil {
		t.Errorf("There was an error creating induced filter!")
	}

	if len(pf0_prime.V0) != 1 {
		t.Errorf("The number of states in F' is %v, but there are 1 elements in induced clique cover.", len(pf0_prime.V0))
	}

}

/*
TestCliqueCover_ToInducedFilter3
Description:
	This function tests the ToInducedFilter() and verifies that it creates the proper number of initial states in the induced filter for Filter 5.
*/
func TestCliqueCover_ToInducedFilter3(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()

	cc, tf := CreateInducedCliqueCover(pf0, pf0)

	// Algorithm
	if !tf {
		t.Errorf("Filed to create induced clique cover!")
	}

	pf0_prime, err := cc.ToInducedFilter()

	if err != nil {
		t.Errorf("There was an error creating induced filter!")
	}

	// for _, v_prime := range pf0_prime.V {
	// 	t.Errorf("Outputs for %v: %v", v_prime, pf0_prime.tau[pf0_prime.V0[0]][v_prime])
	// }

	// if len(pf0_prime.V0) != 1 {
	// 	t.Errorf("The number of states in F' is %v, but there are 1 elements in induced clique cover.", len(pf0_prime.V0))
	// }

}
