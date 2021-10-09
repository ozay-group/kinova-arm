package procrustean

import (
	"testing"
)

func TestProcrusteanFilter_GetProcrusteanFilter1(t *testing.T) {
	// Create Basic PF
	pf1, err := GetProcrusteanFilter(
		[]string{"w0", "w1", "w2", "w3", "w4", "w5", "w6", "w7", "w8", "w9"},
		[]string{"w0"},
		[]string{"a", "b", "c", "e", "f", "g", "h", "o", "p", "q", "r"},
		map[string]map[string][]string{
			"w0": map[string][]string{
				"w1": []string{"o"},
				"w2": []string{"p"},
				"w3": []string{"q"},
				"w4": []string{"r"},
			},
			"w1": map[string][]string{
				"w5": []string{"e"},
				"w9": []string{"b"},
			},
			"w2": map[string][]string{
				"w2": []string{"b"},
				"w8": []string{"f"},
				"w9": []string{"a"},
			},
			"w3": map[string][]string{
				"w3": []string{"c"},
				"w7": []string{"f"},
			},
			"w4": map[string][]string{
				"w4": []string{"a"},
				"w6": []string{"g"},
				"w9": []string{"c"},
			},
			"w5": map[string][]string{
				"w9": []string{"g"},
			},
			"w6": map[string][]string{
				"w4": []string{"g"},
			},
			"w7": map[string][]string{
				"w9": []string{"h"},
			},
			"w8": map[string][]string{
				"w2": []string{"h"},
			},
			"w9": map[string][]string{},
		},
		[]string{"o1", "o2", "o3", "o4", "o5"},
		map[string][]string{
			"w0": []string{"o4"},
			"w1": []string{"o1"},
			"w2": []string{"o1"},
			"w3": []string{"o1"},
			"w4": []string{"o1"},
			"w5": []string{"o2"},
			"w6": []string{"o2"},
			"w7": []string{"o3"},
			"w8": []string{"o3"},
			"w9": []string{"o5"},
		},
	)

	if err != nil {
		t.Errorf("There was an issue creating the Procrustean Filter: %v", err.Error())
	}

	if len(pf1.V) != 10 {
		t.Errorf("Expected for 10 states to be in Procrustean filter but received %v.", len(pf1.V))
	}
}

func ProcrusteanFilter_GetBasic1() (ProcrusteanFilter, error) {
	// Create Basic PF
	pf1, err := GetProcrusteanFilter(
		[]string{"w0", "w1", "w2", "w3", "w4", "w5", "w6", "w7", "w8", "w9"},
		[]string{"w0"},
		[]string{"a", "b", "c", "e", "f", "g", "h", "o", "p", "q", "r"},
		map[string]map[string][]string{
			"w0": map[string][]string{
				"w1": []string{"o"},
				"w2": []string{"p"},
				"w3": []string{"q"},
				"w4": []string{"r"},
			},
			"w1": map[string][]string{
				"w5": []string{"e"},
				"w9": []string{"b"},
			},
			"w2": map[string][]string{
				"w2": []string{"b"},
				"w8": []string{"f"},
				"w9": []string{"a"},
			},
			"w3": map[string][]string{
				"w3": []string{"c"},
				"w7": []string{"f"},
			},
			"w4": map[string][]string{
				"w4": []string{"a"},
				"w6": []string{"e"},
				"w9": []string{"c"},
			},
			"w5": map[string][]string{
				"w9": []string{"g"},
			},
			"w6": map[string][]string{
				"w4": []string{"g"},
			},
			"w7": map[string][]string{
				"w9": []string{"h"},
			},
			"w8": map[string][]string{
				"w2": []string{"h"},
			},
			"w9": map[string][]string{},
		},
		[]string{"o1", "o2", "o3", "o4", "o5"},
		map[string][]string{
			"w0": []string{"o4"},
			"w1": []string{"o1"},
			"w2": []string{"o1"},
			"w3": []string{"o1"},
			"w4": []string{"o1"},
			"w5": []string{"o2"},
			"w6": []string{"o2"},
			"w7": []string{"o3"},
			"w8": []string{"o3"},
			"w9": []string{"o5"},
		},
	)

	return pf1, err
}

func TestProcrusteanFilterState_ReachedBy1(t *testing.T) {

	pf1, err := ProcrusteanFilter_GetBasic1()
	if err != nil {
		t.Errorf("There was an issue getting the basic filter: %v", err.Error())
	}

	observationSequence1 := []string{"r", "a", "e", "g", "c"}
	state1 := pf1.V0[0]
	targetState := pf1.V[9]

	reachedBy, err := targetState.ReachedBy(observationSequence1, state1)
	if err != nil {
		t.Errorf("There was an error running the reached by command: %v", err.Error())
	}

	if !reachedBy {
		t.Errorf("The system incorrectly believes that the observation sequence does not reach the target state!")
	}

}

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
