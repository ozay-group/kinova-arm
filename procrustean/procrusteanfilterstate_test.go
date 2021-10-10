package procrustean

/*
procrusteanfilterstate_test.go
Description:
	Tests the functions related to the ProcrusteanFilterState type in this toolbox.
*/

import (
	"testing"
)

/*
TestProcrusteanFilterState_Equals1
Description:
	This function tests the Equals() function between two different filter states.
*/
func TestProcrusteanFilterState_Equals1(t *testing.T) {
	pfs1 := ProcrusteanFilterState{Name: "s1"}

	pfs2 := ProcrusteanFilterState{Name: "s2"}

	// Algorithm
	if pfs1.Equals(pfs2) {
		t.Errorf("The two states are different but Equals() claims that they aren't!")
	}

}

/*
TestProcrusteanFilterState_Equals2
Description:
	This function tests the Equals() function between two states with the same name filter states.
*/
func TestProcrusteanFilterState_Equals2(t *testing.T) {
	pfs1 := ProcrusteanFilterState{Name: "s1"}

	pfs2 := ProcrusteanFilterState{Name: "s1"}

	// Algorithm
	if !pfs1.Equals(pfs2) {
		t.Errorf("The two states are different but Equals() claims that they aren't!")
	}

}

/*
TestProcrusteanFilterState_AppendIfUniqueTo1
Description:
	The function tests to see if AppendIfUniqueTo successfully adds something unique to a list.
*/
func TestProcrusteanFilterState_AppendIfUniqueTo1(t *testing.T) {
	// Constants
	pfs1 := ProcrusteanFilterState{Name: "s1"}
	pfs2 := ProcrusteanFilterState{Name: "s2"}
	pfs3 := ProcrusteanFilterState{Name: "s3"}

	pfsSlice1 := []ProcrusteanFilterState{pfs1, pfs2}

	// Algorithm
	pfsSlice2 := pfs3.AppendIfUniqueTo(pfsSlice1)

	if len(pfsSlice2) != 3 {
		t.Errorf("The slice pfsSlice2 is supposed to have 3 elements, but it has %v.", len(pfsSlice2))
	}

	if !pfsSlice2[len(pfsSlice2)-1].Equals(pfs3) {
		t.Errorf("The final state in pfsSlice2 was not pfs3!")
	}
}

/*
TestProcrusteanFilterState_AppendIfUniqueTo2
Description:
	The function tests to see if AppendIfUniqueTo successfully ignores adding something to a list when
	it is already in the list..
*/
func TestProcrusteanFilterState_AppendIfUniqueTo2(t *testing.T) {
	// Constants
	pfs1 := ProcrusteanFilterState{Name: "s1"}
	pfs2 := ProcrusteanFilterState{Name: "s2"}
	pfs3 := ProcrusteanFilterState{Name: "s3"}

	pfsSlice1 := []ProcrusteanFilterState{pfs1, pfs2, pfs3}

	// Algorithm
	pfsSlice2 := pfs3.AppendIfUniqueTo(pfsSlice1)

	if len(pfsSlice2) != len(pfsSlice1) {
		t.Errorf("The slice pfsSlice2 is supposed to have the same number of elements as pfsSlice1, but it has %v.", len(pfsSlice2))
	}
}

/*
TestProcrusteanFilterState_In1
Description:
	Tests to see if the In function works when the slice does not contain the targeted state.
*/
func TestProcrusteanFilterState_In1(t *testing.T) {
	// Constants
	pfs1 := ProcrusteanFilterState{Name: "s1"}
	pfs2 := ProcrusteanFilterState{Name: "s2"}
	pfs3 := ProcrusteanFilterState{Name: "s3"}

	pfsSlice1 := []ProcrusteanFilterState{pfs1, pfs2}

	// Algorithm
	if pfs3.In(pfsSlice1) {
		t.Errorf("The state pfs3 is not in pfsSlice1, but the function thinks it is!")
	}
}

/*
TestProcrusteanFilterState_In2
Description:
	Tests to see if the In function works when the slice does contain the targeted state.
*/
func TestProcrusteanFilterState_In2(t *testing.T) {
	// Constants
	pfs1 := ProcrusteanFilterState{Name: "s1"}
	pfs2 := ProcrusteanFilterState{Name: "s2"}
	pfs3 := ProcrusteanFilterState{Name: "s3"}

	pfsSlice1 := []ProcrusteanFilterState{pfs1, pfs2, pfs3}

	// Algorithm
	if !pfs3.In(pfsSlice1) {
		t.Errorf("The state pfs3 is in pfsSlice1, but the function thinks it is not!")
	}
}

/*
TestProcrusteanFilterState_Find1
Description:
	Tests to see if the Find() function works when the slice does not contain the targeted state.
*/
func TestProcrusteanFilterState_Find1(t *testing.T) {
	// Constants
	pfs1 := ProcrusteanFilterState{Name: "s1"}
	pfs2 := ProcrusteanFilterState{Name: "s2"}
	pfs3 := ProcrusteanFilterState{Name: "s3"}

	pfsSlice1 := []ProcrusteanFilterState{pfs1, pfs2}

	// Algorithm
	if pfs3.Find(pfsSlice1) != -1 {
		t.Errorf("The state pfs3 is not in pfsSlice1, but the Find() function thinks it is!")
	}
}

/*
TestProcrusteanFilterState_Find2
Description:
	Tests to see if the Find() function works when the slice does contain the targeted state.
*/
func TestProcrusteanFilterState_Find2(t *testing.T) {
	// Constants
	pfs1 := ProcrusteanFilterState{Name: "s1"}
	pfs2 := ProcrusteanFilterState{Name: "s2"}
	pfs3 := ProcrusteanFilterState{Name: "s3"}

	pfsSlice1 := []ProcrusteanFilterState{pfs1, pfs2, pfs3}

	// Algorithm
	if pfs3.Find(pfsSlice1) != 2 {
		t.Errorf("The state pfs3 is in pfsSlice1, but the function can't properly locate it!")
	}
}

/*
TestProcrusteanFilterState_String1
Description:
	Tests to see if the String() function returns the expected state name.
*/
func TestProcrusteanFilterState_String1(t *testing.T) {
	// Constants
	pfs1 := ProcrusteanFilterState{Name: "s1"}

	// Algorithm
	if pfs1.String() != "s1" {
		t.Errorf("The value of pfs1.String() is unexpected: %v", pfs1)
	}
}

/*
TestProcrusteanFilterState_IsInitial1
Description:
	Determines that an initial state is initial.
*/
func TestProcrusteanFilterState_IsInitial1(t *testing.T) {
	// Constants
	pf1 := GetPFilter1()

	pfs1 := pf1.V[0]

	// Algorithm
	if !pfs1.IsInitial() {
		t.Errorf("The state was initial, but the function could not recognize this!")
	}

}

/*
TestProcrusteanFilterState_IsInitial2
Description:
	Determines that a non-initial state is not initial using the function.
*/
func TestProcrusteanFilterState_IsInitial2(t *testing.T) {
	// Constants
	pf1 := GetPFilter1()

	pfs1 := pf1.V[2]

	// Algorithm
	if pfs1.IsInitial() {
		t.Errorf("The state was NOT initial, but the function could not recognize this!")
	}

}

/*
TestProcrusteanFilterState_ReachesWith1
Description:
	Tests whether or not ReachesWith() gives one state (in a slice) when it should
	for a simple filter.
*/
func TestProcrusteanFilterState_ReachesWith1(t *testing.T) {
	// Constants
	pf1 := GetPFilter2()

	initState := pf1.V0[0]
	simpleString := []string{"a"}

	// Algorithm
	R1 := initState.ReachesWith(simpleString)

	if len(R1) != 1 {
		t.Errorf("The reachable states should only have 1 element but we found %v.", len(R1))
	}

	if !R1[0].Equals(pf1.V[1]) {
		t.Errorf("The reachable state is not what we expect (%v). Received \"%v\".", pf1.V[1], R1[0])
	}

}

/*
TestProcrusteanFilterState_ReachesWith2
Description:
	Tests whether or not ReachesWith() gives zero states (in a slice) when it should
	for a simple filter.
*/
func TestProcrusteanFilterState_ReachesWith2(t *testing.T) {
	// Constants
	pf1 := GetPFilter2()

	initState := pf1.V0[0]
	simpleString := []string{"b"}

	// Algorithm
	R1 := initState.ReachesWith(simpleString)

	if len(R1) != 0 {
		t.Errorf("The reachable states should only have 0 element but we found %v.", len(R1))
	}

}

/*
TestProcrusteanFilterState_ReachesWith3
Description:
	Tests whether or not ReachesWith() gives one states (in a slice) when it receives a
	string slice with two elements.
*/
func TestProcrusteanFilterState_ReachesWith3(t *testing.T) {
	// Constants
	pf1 := GetPFilter2()

	initState := pf1.V0[0]
	simpleString := []string{"a", "a"}

	// Algorithm
	R1 := initState.ReachesWith(simpleString)

	if len(R1) != 1 {
		t.Errorf("The reachable states should only have 0 element but we found %v.", len(R1))
	}

	if !R1[0].Equals(pf1.V[2]) {
		t.Errorf("The reachable state is not what we expect (%v). Received \"%v\".", pf1.V[1], R1[0])
	}

}

/*
TestProcrusteanFilterState_ReachedBy1
Description:
	Tests whether or not ReachedBy() verifies that a state is (correctly) reached with a state.
*/
func TestProcrusteanFilterState_ReachedBy1(t *testing.T) {
	// Constants
	pf1 := GetPFilter2()

	initState := pf1.V0[0]
	targetState := pf1.V[2]
	simpleString := []string{"a", "a"}

	// Algorithm
	if !targetState.ReachedBy(simpleString, initState) {
		t.Errorf("The state %v is reached by the string from %v, but the function claims it does not!", targetState, initState)
	}

}

/*
TestProcrusteanFilterState_ReachedBy2
Description:
	Tests whether or not ReachedBy() verifies that a state is (correctly) reached with a state.
*/
func TestProcrusteanFilterState_ReachedBy2(t *testing.T) {

	pf1, err := ProcrusteanFilter_GetBasic1()
	if err != nil {
		t.Errorf("There was an issue getting the basic filter: %v", err.Error())
	}

	observationSequence1 := []string{"r", "a", "e", "g", "c"}
	state1 := pf1.V0[0]
	targetState := pf1.V[9]

	reachedBy := targetState.ReachedBy(observationSequence1, state1)
	if err != nil {
		t.Errorf("There was an error running the reached by command: %v", err.Error())
	}

	if !reachedBy {
		t.Errorf("The system incorrectly believes that the observation sequence does not reach the target state!")
	}

}

/*
TestProcrusteanFilterState_IntersectionOfStates1
Description:
	Tests whether or not IntersectionOfStates() works with only one slice given.
*/
func TestProcrusteanFilterState_IntersectionOfStates1(t *testing.T) {

	// Constants
	pfs1 := ProcrusteanFilterState{Name: "s1"}
	pfs2 := ProcrusteanFilterState{Name: "s2"}
	pfs3 := ProcrusteanFilterState{Name: "s3"}
	stateSlice1 := []ProcrusteanFilterState{pfs1, pfs2, pfs3}

	// Algorithm
	intersect1 := IntersectionOfStates(stateSlice1)

	if len(stateSlice1) != len(intersect1) {
		t.Errorf("The output of the IntersectionOfStates should have the same length as stateSlice1 (%v), but it does not(%v)!", len(stateSlice1), len(intersect1))
	}
}
