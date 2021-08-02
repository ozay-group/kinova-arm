package OzayGroupExploration

import (
	"testing"
)

func TestTransitionSystem_GetState1(t *testing.T) {
	// Create Simple Transition System
	ts0 := TransitionSystem{}
	ts0.S = []TransitionSystemState{
		TransitionSystemState{1, &ts0},
		TransitionSystemState{2, &ts0},
		TransitionSystemState{3, &ts0},
		TransitionSystemState{4, &ts0},
	}

	// Try to get a state which is outside of the allowable range.
	_, err := ts0.GetState(5)
	if err.Error() != "The input to GetState (5) is not a state from the target transition system." {
		t.Errorf("The correct error was not produced when calling GetState().")
	}
}

func TestTransitionSystem_GetState2(t *testing.T) {
	// Create Simple Transition System
	ts0 := TransitionSystem{}
	ts0.S = []TransitionSystemState{
		TransitionSystemState{1, &ts0},
		TransitionSystemState{2, &ts0},
		TransitionSystemState{3, &ts0},
		TransitionSystemState{4, &ts0},
	}

	// Try to get a state which is outside of the allowable range.
	tempState, err := ts0.GetState(2)
	if err != nil {
		t.Errorf("The correct error was not produced when calling GetState().")
	}

	if tempState.Value != 2 {
		t.Errorf("The value of the correct state (2) was not saved in the state.")
	}
}

func TestTransitionSystem_GetTransitionSystem1(t *testing.T) {
	ts0, err := GetTransitionSystem(
		[]int{1, 2, 3}, []int{1, 2},
		map[int][][]int{
			1: [][]int{
				[]int{1},
				[]int{2},
			},
			2: [][]int{
				[]int{1, 2},
				[]int{2, 3},
			},
			3: [][]int{
				[]int{3},
				[]int{2},
			},
		},
		[]int{1},
		[]string{"A", "B", "C", "D"},
		map[int][]string{
			1: []string{"A"},
			2: []string{"B", "D"},
			3: []string{"C", "D"},
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
