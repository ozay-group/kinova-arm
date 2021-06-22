package OzayGroupExploration

import "testing"

func TestTransitionSystem_GetState1(t *testing.T) {
	// Create Simple Transition System
	ts0 := TransitionSystem{
		S: []int{1, 2, 3, 4},
	}

	// Try to get a state which is outside of the allowable range.
	_, err := ts0.GetState(5)
	if err.Error() != "The input to GetState (5) is not a state from the target transition system." {
		t.Errorf("The correct error was not produced when calling GetState().")
	}
}

func TestTransitionSystem_GetState2(t *testing.T) {
	// Create Simple Transition System
	ts0 := TransitionSystem{
		S: []int{1, 2, 3, 4},
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
