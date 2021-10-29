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

/*
TestProcrusteanFilterState_IntersectionOfStates2
Description:
	Tests whether or not IntersectionOfStates() works with three slices given.
*/
func TestProcrusteanFilterState_IntersectionOfStates2(t *testing.T) {

	// Constants
	pfs1 := ProcrusteanFilterState{Name: "s1"}
	pfs2 := ProcrusteanFilterState{Name: "s2"}
	pfs3 := ProcrusteanFilterState{Name: "s3"}
	stateSlice1 := []ProcrusteanFilterState{pfs1, pfs2, pfs3}

	pfs4 := ProcrusteanFilterState{Name: "cheddar"}
	pfs5 := ProcrusteanFilterState{Name: "gruyere"}
	stateSlice2 := []ProcrusteanFilterState{pfs3, pfs4, pfs5}

	stateSlice3 := []ProcrusteanFilterState{pfs1, pfs3, pfs4, pfs5}

	// Algorithm
	intersect1 := IntersectionOfStates(stateSlice1, stateSlice2, stateSlice3)

	if len(intersect1) != 1 {
		t.Errorf("The output of the IntersectionOfStates should have 1 element but it has length %v!", len(intersect1))
	}
}

/*
TestProcrusteanFilterState_LanguageWithLength1
Description:
	Tests whether or not LanguageWithLength() works with length 1 for the simple 4 state system. Should only return one state frominitial state.
*/
func TestProcrusteanFilterState_LanguageWithLength1(t *testing.T) {

	// Constants
	pf0 := GetPFilter2()

	initState := pf0.V0[0]

	// Algorithm
	Language0, err := initState.LanguageWithLength(1)
	if err != nil {
		t.Errorf("There was an error when running LanguageWithMaxLength(): %v", err)
	}

	if len(Language0) != 1 {
		t.Errorf("The output of the LanguageWithMaxLength should have 1 element but it has length %v!", len(Language0))
	}

	if Language0[0].s[0] != "a" {
		t.Errorf("The only extension that should be in Language0 is \"a\", but we found \"%v\".", Language0[0].s[0])
	}
}

/*
TestProcrusteanFilterState_LanguageWithMaxLength2
Description:
	Tests whether or not LanguageWithMaxLength() works with length 1 for the simple 4 state system. Should only return one state frominitial state.
*/
func TestProcrusteanFilterState_LanguageWithLength2(t *testing.T) {

	// Constants
	pf0 := GetPFilter2()

	initState := pf0.V0[0]

	// Algorithm
	Language0, err := initState.LanguageWithLength(2)
	if err != nil {
		t.Errorf("There was an error when running LanguageWithMaxLength(): %v", err)
	}

	if len(Language0) != 2 {
		t.Errorf("The output of the LanguageWithMaxLength should have 1 element but it has length %v!", len(Language0))
	}

	if (Language0[0].s[0] != "a") || (Language0[1].s[0] != "a") {
		t.Errorf("All extension in Language0 should begin with \"a\", but we found \"%v\" and \"%v\".", Language0[0].s[0], Language0[1].s[0])
	}

	if (Language0[0].s[1] != "a") || (Language0[1].s[1] != "b") {
		t.Errorf("The extensions in Language0 should have different next values.")
	}
}

/*
TestProcrusteanFilterState_LanguageWithMaxLength3
Description:
	Tests whether or not LanguageWithMaxLength() works with length 3 for the simple 4 state system. Should return two strings for initial state.
*/
func TestProcrusteanFilterState_LanguageWithLength3(t *testing.T) {

	// Constants
	pf0 := GetPFilter2()

	initState := pf0.V0[0]

	// Algorithm
	Language0, err := initState.LanguageWithLength(3)
	if err != nil {
		t.Errorf("There was an error when running LanguageWithMaxLength(): %v", err)
	}

	if len(Language0) != 2 {
		t.Errorf("The output of the LanguageWithMaxLength should have 1 element but it has length %v!", len(Language0))
	}
}

/*
TestProcrusteanFilterState_LanguageWithMaxLength1
Description:
	Tests whether or not LanguageWithMaxLength() works with length 10 for the Example 1 (Figure 4 in paper). Should return two executions only.
*/
func TestProcrusteanFilterState_LanguageWithMaxLength1(t *testing.T) {

	// Constants
	pf0 := GetPFilter1()

	w1 := pf0.V[1]

	// Algorithm
	Language0, err := w1.LanguageWithMaxLength(10)
	if err != nil {
		t.Errorf("There was an error when running LanguageWithMaxLength(): %v", err)
	}

	if len(Language0) != 4 {
		for _, extension := range Language0 {
			t.Errorf("%v", extension)
		}
		t.Errorf("The output of the LanguageWithMaxLength should have 2 element but it has length %v!", len(Language0))
	}
}

/*
TestProcrusteanFilterState_S1
Description:
	This algorithm tests to make sure that the function S correctly identifies all initial states that can reach a given target state.
*/
func TestProcrusteanFilterState_S1(t *testing.T) {
	// Constants
	pf0 := GetPFilter2()

	s3 := pf0.V[2]

	// Algorithm
	initialStatesThatReachs3 := s3.S()

	if len(initialStatesThatReachs3) != 1 {
		t.Errorf("The only initial state in pf0 should reach state S3, but the function claims that %v states reach s3.", len(initialStatesThatReachs3))
	}

}

/*
TestProcrusteanFilterState_CorrespondsWith1
Description:
	This algorithm tests to make sure that the function CorrespondsWith()
	correctly identifies that the same state
	corresponds with itself according to the definition in the WAFR paper.
*/
func TestProcrusteanFilterState_CorrespondsWith1(t *testing.T) {
	// Constants
	pf0 := GetPFilter2()

	s3 := pf0.V[2]
	//s4 := pf0.V[3]

	// Algorithm

	if !s3.CorrespondsWith(s3) {
		t.Errorf("The two states should correspond with each other, but the function claims that they are not!")
	}

}

/*
TestProcrusteanFilterState_CorrespondsWith2
Description:
	This algorithm tests to make sure that the function CorrespondsWith()
	correctly identifies that two states in the same filter (which are reachable)
	are corresponding according to the definition in the WAFR paper.
*/
func TestProcrusteanFilterState_CorrespondsWith2(t *testing.T) {
	// Constants
	pf0 := GetPFilter2()

	s3 := pf0.V[2]
	s4 := pf0.V[3]

	// Algorithm

	if s3.CorrespondsWith(s4) {
		t.Errorf("The two states should not correspond with each other, but the function claims that they do!")
	}

}

/*
TestProcrusteanFilterState_IsCompatibleWith1
Description:
	Verifies that two vertices which we know are incompatible in example 1 are compatible according to the function.
*/
func TestProcrusteanFilterState_IsCompatibleWith1(t *testing.T) {
	// Constants
	pf0 := GetPFilter1()

	w1 := pf0.V[1]
	w2 := pf0.V[2]

	// Algorithm
	if tf, _ := w1.IsCompatibleWith(w2); tf {
		t.Errorf("The states \"%v\" and \"%v\" are not compatible, but the function claims they are.", w1, w2)
	}

}

/*
TestProcrusteanFilterState_IsCompatibleWith2
Description:
	Verifies that two vertices which we know are compatible in example 1 are compatible according to the function.
*/
func TestProcrusteanFilterState_IsCompatibleWith2(t *testing.T) {
	// Constants
	pf0 := GetPFilter4()

	w2 := pf0.V[2]
	w3 := pf0.V[3]

	// Algorithm
	if tf, err := w2.IsCompatibleWith(w3); !tf {
		t.Errorf("The states \"%v\" and \"%v\" are compatible, but the function claims they are not. err = %v", w2, w3, err)
	}

}

/*
TestProcrusteanFilterState_IsMutuallyCompatibleSet1
Description:
	Verifies that two vertices which we know are compatible in example 4 form a mutually compatible set.
*/
func TestProcrusteanFilterState_IsMutuallyCompatibleSet1(t *testing.T) {
	// Constants
	pf0 := GetPFilter4()

	w2 := pf0.V[2]
	w3 := pf0.V[3]

	// Algorithm
	if !IsMutuallyCompatibleSet([]ProcrusteanFilterState{w2, w3}) {
		t.Errorf("The set is mutually compatible, but the function claims it is not.")
	}

}

/*
TestProcrusteanFilterState_IsMutuallyCompatibleSet2
Description:
	Verifies that two vertices which we know are NOT compatible in example 4 form a mutually compatible set.
*/
func TestProcrusteanFilterState_IsMutuallyCompatibleSet2(t *testing.T) {
	// Constants
	pf0 := GetPFilter4()

	w1 := pf0.V[1]
	w3 := pf0.V[3]

	// Algorithm
	if IsMutuallyCompatibleSet([]ProcrusteanFilterState{w1, w3}) {
		t.Errorf("The set is NOT mutually compatible, but the function claims it is not.")
	}

}

/*
TestProcrusteanFilterState_IsMutuallyCompatibleSet3
Description:
	Verifies that two vertices which we know are compatible in example 5 form a mutually compatible set.
*/
func TestProcrusteanFilterState_IsMutuallyCompatibleSet3(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()

	w1 := pf0.V[1]
	w2 := pf0.V[2]

	w5 := pf0.V[5]
	w6 := pf0.V[6]

	// Algorithm
	if !IsMutuallyCompatibleSet([]ProcrusteanFilterState{w1, w2}) || !IsMutuallyCompatibleSet([]ProcrusteanFilterState{w5, w6}) {
		t.Errorf("These sets are known to be mutually compatible, but the function claims they are not.")
	}

}

/*
TestProcrusteanFilterState_FormZipperConstraint1
Description:
	Verifies that two sets of states which we know are compatible in example 5 form a zipper constraint.
*/
func TestProcrusteanFilterState_FormZipperConstraint1(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()

	set1 := pf0.V[1:3]
	set2 := pf0.V[5:7]

	y := "a"

	// Algorithm
	if !FormZipperConstraint(set1, set2, y) {
		t.Errorf("These sets are known to form a zipper constraint, but the function claims they do not.")
	}

}

/*
TestProcrusteanFilterState_FormZipperConstraint2
Description:
	Verifies that ANOTHER two sets of states which we know are compatible in example 5 form a zipper constraint.
*/
func TestProcrusteanFilterState_FormZipperConstraint2(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()

	set1 := pf0.V[3:5]
	set2 := pf0.V[6:8]

	y := "b"

	// Algorithm
	if !FormZipperConstraint(set1, set2, y) {
		t.Errorf("These sets are known to form a zipper constraint, but the function claims they do not.")
	}

}

/*
TestProcrusteanFilterState_Post1
Description:
	Tests when Post produces a single element in response. This is the version of Post with two inputs.
*/
func TestProcrusteanFilterState_Post1(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()
	v0 := pf0.V[0]
	y0 := pf0.Y[0]

	// Algorithm
	nextStates, _ := Post(v0, y0)

	if tf, _ := SliceEquals(nextStates, []ProcrusteanFilterState{pf0.V[1]}); !tf {
		t.Errorf("Expected for the post to result in state V[1] being reached only, but there are %v different states reached.", len(nextStates))
	}
}

/*
TestProcrusteanFilterState_Post2
Description:
	Tests when Post produces four elements in response. This is the version of Post with one inputs.
*/
func TestProcrusteanFilterState_Post2(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()
	v0 := pf0.V[0]

	// Algorithm
	nextStates, _ := Post(v0)

	if tf, _ := SliceEquals(nextStates, pf0.V[1:5]); !tf {
		t.Errorf("Expected for the post to result in state V[1] being reached only, but there are %v different states reached.", len(nextStates))
	}
}

/*
TestProcrusteanFilterState_Post3
Description:
	Tests when Post produces zero element sin response. This is the version of Post with two inputs.
*/
func TestProcrusteanFilterState_Post3(t *testing.T) {
	// Constants
	pf0 := GetPFilter5()
	v1 := pf0.V[1]
	y1 := pf0.Y[1]

	// Algorithm
	nextStates, _ := Post(v1, y1)

	if tf, _ := SliceEquals(nextStates, []ProcrusteanFilterState{}); !tf {
		t.Errorf("Expected for the post to result in state V[1] being reached only, but there are %v different states reached.", len(nextStates))
	}
}

/*
TestProcrusteanFilterState_HasExtension1
Description:
	Tests whether or not the function correctly detects that a single string extension is valid for a simple filter.
*/
func TestProcrusteanFilterState_HasExtension1(t *testing.T) {
	// Constants
	pf0 := GetPFilter2()

	ec0 := ExtensionCandidate{s: []string{pf0.Y[0]}, Filter: &pf0}
	v0 := pf0.V[0]

	// Algorithm
	if !v0.HasExtension(ec0) {
		t.Errorf("The extension candidate is truly an extension but the function claims that it does not.")
	}
}

/*
TestProcrusteanFilterState_HasExtension2
Description:
	Tests whether or not the function correctly detects that an empty string is an extension valid for any filter state.
*/
func TestProcrusteanFilterState_HasExtension2(t *testing.T) {
	// Constants
	pf0 := GetPFilter2()

	ec0 := ExtensionCandidate{s: []string{pf0.Y[0]}, Filter: &pf0}
	v0 := pf0.V[0]
	v1 := pf0.V[1]

	// Algorithm
	if !v0.HasExtension(ec0) {
		t.Errorf("The empty extension candidate is an extension but the function claims that it does not.")
	}

	if !v1.HasExtension(ec0) {
		t.Errorf("The empty extension candidate is an extension but the function claims that it does not.")
	}
}
