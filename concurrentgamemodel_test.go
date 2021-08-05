package OzayGroupExploration

import (
	"testing"
)

func CreateSimpleCGM() (ConcurrentGameModel, error) {
	cgm2, err := CreateConcurrentGameModel(
		[]string{"Agent1", "Agent2"},
		[]string{"Rome", "Florence", "Venice"},
		[]string{"Drunk", "Sober"},
		[]string{"Ride Horse", "Take Boat", "Follow Legion"},
		map[string]map[string][]string{
			"Agent1": map[string][]string{
				"Rome":     []string{"Ride Horse", "Take Boat", "Follow Legion"},
				"Florence": []string{"Ride Horse", "Take Boat"},
				"Venice":   []string{"Take Boat"},
			},
			"Agent2": map[string][]string{
				"Rome":     []string{"Ride Horse", "Take Boat", "Follow Legion"},
				"Florence": []string{"Ride Horse"},
				"Venice":   []string{"Take Boat", "Follow Legion"},
			},
		},
		map[string]map[string]string{
			"Rome": map[string]string{
				"Ride Horse, Ride Horse":       "Florence",
				"Ride Horse, Take Boat":        "Florence",
				"Ride Horse, Follow Legion":    "Florence",
				"Take Boat, Ride Horse":        "Venice",
				"Take Boat, Take Boat":         "Venice",
				"Take Boat, Follow Legion":     "Venice",
				"Follow Legion, Ride Horse":    "Rome",
				"Follow Legion, Take Boat":     "Rome",
				"Follow Legion, Follow Legion": "Rome",
			},
			"Florence": map[string]string{
				"Ride Horse, Ride Horse":       "Rome",
				"Ride Horse, Take Boat":        "Venice",
				"Ride Horse, Follow Legion":    "Florence",
				"Take Boat, Ride Horse":        "Florence",
				"Take Boat, Take Boat":         "Florence",
				"Take Boat, Follow Legion":     "Florence",
				"Follow Legion, Ride Horse":    "Florence",
				"Follow Legion, Take Boat":     "Florence",
				"Follow Legion, Follow Legion": "Rome",
			},
			"Venice": map[string]string{
				"Ride Horse, Ride Horse":       "Rome",
				"Ride Horse, Take Boat":        "Rome",
				"Ride Horse, Follow Legion":    "Venice",
				"Take Boat, Ride Horse":        "Florence",
				"Take Boat, Take Boat":         "Rome",
				"Take Boat, Follow Legion":     "Florence",
				"Follow Legion, Ride Horse":    "Florence",
				"Follow Legion, Take Boat":     "Florence",
				"Follow Legion, Follow Legion": "Rome",
			},
		},
		map[string][]string{
			"Drunk": []string{"Rome"},
			"Sober": []string{"Florence", "Venice"},
		},
	)

	return cgm2, err
}

func TestConcurrentGameModel_Constructor1(t *testing.T) {
	// Create Simple Empty Concurrent Game Model
	cgm1 := ConcurrentGameModel{}

	// Try to get a state which is outside of the allowable range.
	if cgm1.o != nil {
		t.Errorf("Expected for uninitialized CGM to have nil transition function. But it is not nil!")
	}

}

func TestConcurrentGameModel_CreateConcurrentGameModel1(t *testing.T) {
	// Create Simple Empty Concurrent Game Model
	cgm2, err := CreateSimpleCGM()
	if err != nil {
		t.Errorf("There was an error while creating simple CGM: %v", err.Error())
	}

	// Try to get a state which is outside of the allowable range.
	if cgm2.o == nil {
		t.Errorf("Expected for uninitialized CGM to have non-empty transition function. But it is nil!")
	}

}
