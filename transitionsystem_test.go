package OzayGroupExploration

import (
	"testing"
)

func TestTransitionSystem_GetState1(t *testing.T) {
	// Create Simple Transition System
	ts0 := TransitionSystem{}
	ts0.S = []TransitionSystemState{
		TransitionSystemState{"1", &ts0},
		TransitionSystemState{"2", &ts0},
		TransitionSystemState{"3", &ts0},
		TransitionSystemState{"4", &ts0},
	}

	// Try to get a state which is outside of the allowable range.
	if len(ts0.S) != 4 {
		t.Errorf("There are not four states left.")
	}
}

func TestTransitionSystem_GetState2(t *testing.T) {
	// Create Simple Transition System
	ts0 := TransitionSystem{}
	ts0.S = []TransitionSystemState{
		TransitionSystemState{"1", &ts0},
		TransitionSystemState{"2", &ts0},
		TransitionSystemState{"3", &ts0},
		TransitionSystemState{"4", &ts0},
	}

	// Try to get a state which is outside of the allowable range.
	tempState := ts0.S[1]
	if tempState.Name != "2" {
		t.Errorf("The value of the correct state (2) was not saved in the state.")
	}
}

func TestTransitionSystem_GetTransitionSystem1(t *testing.T) {
	ts0, err := GetTransitionSystem(
		[]string{"1", "2", "3"}, []string{"1", "2"},
		map[string]map[string][]string{
			"1": map[string][]string{
				"1": []string{"1"},
				"2": []string{"2"},
			},
			"2": map[string][]string{
				"1": []string{"1", "2"},
				"2": []string{"2", "3"},
			},
			"3": map[string][]string{
				"1": []string{"3"},
				"2": []string{"2"},
			},
		},
		[]string{"1"},
		[]string{"A", "B", "C", "D"},
		map[string][]string{
			"1": []string{"A"},
			"2": []string{"B", "D"},
			"3": []string{"C", "D"},
		},
	)

	if err != nil {
		t.Errorf("Unexpected error: %v", err)
	}

	// Tests
	if len(ts0.AP) != 4 {
		t.Errorf("The number of atomic propositions was expected to be 4, but it was %v.", len(ts0.AP))
	}
}

/*
TestTransitionSystem_Equals1
Description:
	Tests if the Equals() member function for AtomicProposition works.
*/
func TestTransitionSystem_Equals1(t *testing.T) {
	// Constants
	ap1 := AtomicProposition{Name: "A"}
	ap2 := AtomicProposition{Name: "B"}
	ap3 := AtomicProposition{Name: "A"}

	if ap1.Equals(ap2) {
		t.Errorf("ap1 (%v) is supposed to be different from ap2 (%v).", ap1.Name, ap2.Name)
	}

	if !ap1.Equals(ap3) {
		t.Errorf("ap1 (%v) is supposed to be the same as ap3 (%v).", ap1.Name, ap3.Name)
	}

}

/*
GetSimpleTS
Description:
	Get a transition system to test satisfies.
*/
func GetSimpleTS1() TransitionSystem {
	ts0, _ := GetTransitionSystem(
		[]string{"1", "2", "3"}, []string{"1", "2"},
		map[string]map[string][]string{
			"1": map[string][]string{
				"1": []string{"1"},
				"2": []string{"2"},
			},
			"2": map[string][]string{
				"1": []string{"1", "2"},
				"2": []string{"2", "3"},
			},
			"3": map[string][]string{
				"1": []string{"3"},
				"2": []string{"2"},
			},
		},
		[]string{"1"},
		[]string{"A", "B", "C", "D"},
		map[string][]string{
			"1": []string{"A"},
			"2": []string{"B", "D"},
			"3": []string{"C", "D"},
		},
	)

	return ts0
}

/*
TestTransitionSystem_Satisfies1
Description:
	Tests if the Satisfies() member function correctly identifies when the system
	satisfies a given transition system.
*/
func TestTransitionSystem_Satisfies1(t *testing.T) {
	// Constants
	ts1 := GetSimpleTS1()
	ap2 := AtomicProposition{Name: "B"}

	// Test
	state2 := ts1.S[1]

	tf, err := state2.Satisfies(ap2)
	if err != nil {
		t.Errorf("There was an error while testing satisfies: %v", err.Error())
	}

	if !tf {
		t.Errorf("ap1 (%v) is supposed to be satisfied by ts1.", ap2.Name)
	}

}

/*
TestTransitionSystem_Satisfies2
Description:
	Tests if the Satisfies() member function correctly identifies when the system
	satisfies a given transition system.
*/
func TestTransitionSystem_Satisfies2(t *testing.T) {
	// Constants
	ts1 := GetSimpleTS1()
	ap2 := AtomicProposition{Name: "B"}

	// Test
	state2 := ts1.S[0]

	tf, err := state2.Satisfies(ap2)
	if err != nil {
		t.Errorf("There was an error while testing satisfies: %v", err.Error())
	}

	if tf {
		t.Errorf("ap1 (%v) is supposed to NOT be satisfied by ts1.", ap2.Name)
	}

}

/*
TestTransitionSystemState_Post1
Description:
	Tests if the Post() member function correctly identifies when the system
	satisfies a given transition system.
*/
func TestTransitionSystemState_Post1(t *testing.T) {
	// Constants
	ts1 := GetSimpleTS1()

	// Test
	state2 := ts1.S[1]

	nextStates, err := Post(state2, "1")
	if err != nil {
		t.Errorf("There was an error while testing Post: %v", err.Error())
	}

	if len(nextStates) != 2 {
		t.Errorf("Expected there to be 2 options for next state but received %v options. %v", len(nextStates), nextStates[0])
	}

}

/*
TestTransitionSystemState_Post2
Description:
	Tests if the Post() member function correctly creates the ancestors
	when there is no action given.
*/
func TestTransitionSystemState_Post2(t *testing.T) {
	// Constants
	ts1 := GetSimpleTS1()

	// Test
	state1 := ts1.S[0]
	state3 := ts1.S[2]

	nextStates, err := Post(state1)
	if err != nil {
		t.Errorf("There was an error while testing Post: %v", err.Error())
	}

	if len(nextStates) != 2 {
		t.Errorf("Expected there to be 2 options for next state but received %v options. %v", len(nextStates), nextStates[0])
	}

	if state3.In(nextStates) {
		t.Errorf("Expected for state3 to not be in the post result, but it was found there!")
	}

}

/*
TestTransitionSystemState_Post3
Description:
	Tests if the Post() member function correctly creates a post set which
	DOES NOT include all states in the system.
*/
func TestTransitionSystemState_Post3(t *testing.T) {
	// Constants
	ts1 := GetSimpleTS1()

	// Test
	state2 := ts1.S[1]

	nextStates, err := Post(state2)
	if err != nil {
		t.Errorf("There was an error while testing Post: %v", err.Error())
	}

	if len(nextStates) != 3 {
		t.Errorf("Expected there to be 3 options for next state but received %v options. %v", len(nextStates), nextStates[0])
	}

}

/*
TestTransitionSystemState_Pre1
Description:
	Tests if the Pre() member function correctly finds the proper state when given a
	state and action with only one possible predecessor.
*/
func TestTransitionSystemState_Pre1(t *testing.T) {
	// Constants
	ts1 := GetSimpleTS1()

	// Test
	state3 := ts1.S[2]

	predecessors, err := Pre(state3, "1")
	if err != nil {
		t.Errorf("There was an error while testing Post: %v", err.Error())
	}

	if len(predecessors) != 1 {
		t.Errorf("Expected there to be 1 options for next state but received %v options. %v", len(predecessors), predecessors[0])
	}

	if !predecessors[0].Equals(state3) {
		t.Errorf("Expected for the predecessor to be state3 but received the state%v.", predecessors[0].Name)
	}

}

/*
TestTransitionSystemState_Pre2
Description:
	Tests if the Pre() member function correctly finds the proper state when given a
	state only.
*/
func TestTransitionSystemState_Pre2(t *testing.T) {
	// Constants
	ts1 := GetSimpleTS1()

	// Test
	state3 := ts1.S[2]

	predecessors, err := Pre(state3)
	if err != nil {
		t.Errorf("There was an error while testing Post: %v", err.Error())
	}

	if len(predecessors) != 2 {
		t.Errorf("Expected there to be 2 options for next state but received %v options. %v", len(predecessors), predecessors[0])
	}

	if !ts1.S[1].In(predecessors) {
		t.Errorf("Expected for state2 to be in predecessors, but it was not!")
	}

	if ts1.S[0].In(predecessors) {
		t.Errorf("Expected for state1 to NOT be in predecessors, but it was!")
	}

}

/*
GetSimpleTS2
Description:
	Get a transition system to test Pre and Post.
	It should have states that generates empty sets for Pre() and Post() respectively.
*/
func GetSimpleTS2() TransitionSystem {
	ts0, _ := GetTransitionSystem(
		[]string{"1", "2", "3", "4", "5"}, []string{"1", "2"},
		map[string]map[string][]string{
			"1": map[string][]string{
				"1": []string{"1"},
				"2": []string{"2"},
			},
			"2": map[string][]string{
				"1": []string{"1", "2", "4"},
				"2": []string{"2", "3"},
			},
			"3": map[string][]string{
				"1": []string{"3"},
				"2": []string{"2", "4"},
			},
			"4": map[string][]string{
				"1": []string{},
				"2": []string{},
			},
			"5": map[string][]string{
				"1": []string{"4"},
				"2": []string{"1", "2", "3"},
			},
		},
		[]string{"1"},
		[]string{"A", "B", "C", "D"},
		map[string][]string{
			"1": []string{"A"},
			"2": []string{"B", "D"},
			"3": []string{"C", "D"},
			"4": []string{"A", "C"},
			"5": []string{"A", "B", "C", "D"},
		},
	)

	return ts0
}

/*
TestTransitionSystemState_Post4
Description:
	Tests if the Post() member function correctly finds an empty set when
	given the right system and inputs.
*/
func TestTransitionSystemState_Post4(t *testing.T) {
	// Constants
	ts1 := GetSimpleTS2()

	// Test
	state4 := ts1.S[3]

	ancestors, err := Post(state4)
	if err != nil {
		t.Errorf("There was an error while testing Post: %v", err.Error())
	}

	if len(ancestors) != 0 {
		t.Errorf("Expected there to be 0 options for next state but received %v options. %v", len(ancestors), ancestors[0])
	}

}

/*
TestTransitionSystemState_Pre3
Description:
	Tests if the Pre() member function correctly finds an empty set when
	given.
*/
func TestTransitionSystemState_Pre3(t *testing.T) {
	// Constants
	ts1 := GetSimpleTS2()

	// Test
	state5 := ts1.S[4]

	predecessors, err := Pre(state5)
	if err != nil {
		t.Errorf("There was an error while testing Post: %v", err.Error())
	}

	if len(predecessors) != 0 {
		t.Errorf("Expected there to be 0 options for next state but received %v options. %v", len(predecessors), predecessors[0])
	}

}
