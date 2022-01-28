/*
procrusteanfilterstate.go
Description:
	An implementation of the Procrustean filter as described by Yulin Zhang and
	Dylan A. Shell in their 2020 WAFR paper "Cover Combinatorial Filters and
	their Minimization Problem".
*/

package procrustean

import (
	"errors"
	"fmt"

	oze "github.com/kwesiRutledge/OzayGroupExploration"

	combinations "github.com/mxschmitt/golang-combinations"
)

type ProcrusteanFilterState struct {
	Name   string
	Filter *ProcrusteanFilter
}

/*
ReachedBy
Description:

Assumption:
	Assumes that the state stateIn was checked before running.
	Similarly that the observation sequence was checked.
*/
func (finalState ProcrusteanFilterState) ReachedBy(observationSequence []string, initialState ProcrusteanFilterState) bool {
	// Constants

	// Check Observation Sequence
	return finalState.In(initialState.ReachesWith(observationSequence))
}

/*
ReachesWith
Description:

	An implemenation of V_stateIn(F,s) from Definition 2.
Assumption:
	Assumes that the state stateIn was checked before running.
Usage:
	R := stateIn.ReachesWith(s)
*/
func (stateIn ProcrusteanFilterState) ReachesWith(observationSequence []string) []ProcrusteanFilterState {
	// Constants

	// Input Processing
	if len(observationSequence) == 0 {
		return []ProcrusteanFilterState{stateIn}
	}

	// Algorithm

	// Iterate through Observarion Sequence to Identify Whether or Not the Final State is Reached.
	var statesReachedAtTime [][]ProcrusteanFilterState
	statesReachedAtTime = append(statesReachedAtTime, []ProcrusteanFilterState{stateIn})
	for tau, observationAtTau := range observationSequence {
		// Find Which transitions exist between the current state
		currentStates := statesReachedAtTime[tau]
		var successorStates []ProcrusteanFilterState
		for _, currentState := range currentStates {
			//and some other state, when given the current observation (observationAtTau)
			var successorsOfCurrentState []ProcrusteanFilterState
			tempPost, _ := Post(currentState, observationAtTau)

			// Append values from tempPost to successorStates
			for _, postState := range tempPost {
				successorsOfCurrentState = postState.AppendIfUniqueTo(successorsOfCurrentState)
				successorStates = postState.AppendIfUniqueTo(successorStates)
			}

		}
		statesReachedAtTime = append(statesReachedAtTime, successorStates)
		// fmt.Println(fmt.Sprintf("Time = %v ; ", tau))
		// fmt.Println(successorStates)
	}
	return statesReachedAtTime[len(observationSequence)]

}

/*
Post
Description:
	Finds the set of states that can follow a given state (or set of states).
	Specifically for ProcrusteanFilter objects.
Usage:
	ancestorStates, err := Post( initialState )
	ancestorStates, err := Post( initialState , actionIn )
*/
func Post(SorSY ...interface{}) ([]ProcrusteanFilterState, error) {
	switch len(SorSY) {
	case 1:
		// Only State Is Given
		stateIn, ok := SorSY[0].(ProcrusteanFilterState)
		if !ok {
			return []ProcrusteanFilterState{}, errors.New("The first input to post is not of type TransitionSystemState.")
		}

		Filter := stateIn.Filter
		Y := Filter.Y

		var nextStates []ProcrusteanFilterState
		var tempPost []ProcrusteanFilterState
		var err error

		for _, y := range Y {
			tempPost, err = Post(stateIn, y)
			if err != nil {
				return []ProcrusteanFilterState{}, err
			}
			for _, postElt := range tempPost {
				nextStates = postElt.AppendIfUniqueTo(nextStates)
			}
		}

		return nextStates, nil

	case 2:
		// State and Action is Given
		stateIn, ok := SorSY[0].(ProcrusteanFilterState)
		if !ok {
			return []ProcrusteanFilterState{}, errors.New("The first input to post is not of type TransitionSystemState.")
		}

		yIn, ok := SorSY[1].(string)
		if !ok {
			return []ProcrusteanFilterState{}, errors.New("The second input to post is not of type string!")
		}

		// Get Transition value
		Filter := stateIn.Filter
		TofS := Filter.tau[stateIn]

		var nextStates []ProcrusteanFilterState

		for nextState, YSubset := range TofS {
			if _, tf := oze.FindStringInSlice(yIn, YSubset); tf {
				nextStates = nextState.AppendIfUniqueTo(nextStates)
			}
		}

		return nextStates, nil
	}

	// Return error
	return []ProcrusteanFilterState{}, errors.New(fmt.Sprintf("Unexpected number of inputs to post (%v).", len(SorSY)))
}

/*
Equals
Description:
	Returns true if the two ProcrusteanFilterState objects have the same name.
*/
func (stateIn ProcrusteanFilterState) Equals(stateTwo ProcrusteanFilterState) bool {
	return stateIn.Name == stateTwo.Name
}

/*
AppendIfUniqueTo
Description:
	Appends the state stateIn to the slice stateSlice only if stateSlice doesn't already contain it.
*/
func (stateIn ProcrusteanFilterState) AppendIfUniqueTo(stateSlice []ProcrusteanFilterState) []ProcrusteanFilterState {
	// Constants

	// Algorithm
	if stateIn.In(stateSlice) {
		return stateSlice
	} else {
		return append(stateSlice, stateIn)
	}

}

/*
Find
Description:
	This algorithm will:
	- if stateIn is in stateSlice, this returns the index where stateIn is found in stateSlice
	- if stateIn is NOT in stateSlice, this returns -1
*/
func (stateIn ProcrusteanFilterState) Find(stateSlice []ProcrusteanFilterState) int {
	// Constants

	// Algorithm
	for stateIndex, tempState := range stateSlice {
		if tempState.Equals(stateIn) {
			return stateIndex
		}
	}

	return -1
}

/*
In
Description:
	This function returns a boolean explaining if a state is in the slice stateSlice or not.
*/
func (stateIn ProcrusteanFilterState) In(stateSlice []ProcrusteanFilterState) bool {
	return stateIn.Find(stateSlice) != -1
}

/*
HasExtension
Description:
	Determines if the ProcrusteanFilterState object has an extension which is ExtensionCandidate type.
*/
func (stateIn ProcrusteanFilterState) HasExtension(extensionIn ExtensionCandidate) bool {
	// Input Checking

	// Constants

	// Algorithm
	statesAtk := []ProcrusteanFilterState{stateIn}
	var statesAtkPlus1 []ProcrusteanFilterState

	for _, observation := range extensionIn.s {
		// Make the set of states at time t+1 a bit larger.
		statesAtkPlus1 = []ProcrusteanFilterState{}

		// Compute Post for each state in stateAtk
		for _, stateAtk := range statesAtk {
			tempPost, _ := Post(stateAtk, observation)

			for _, stateAtkPlus1 := range tempPost {
				statesAtkPlus1 = stateAtkPlus1.AppendIfUniqueTo(statesAtkPlus1)
			}
		}
	}

	return len(statesAtkPlus1) != 0
}

/*
String
Description:
	Returns the name of the state.
*/
func (stateIn ProcrusteanFilterState) String() string {
	return stateIn.Name
}

/*
IsInitial
Description:
	Returns a boolean and describes whether or not the state is an initial state.
*/
func (stateIn ProcrusteanFilterState) IsInitial() bool {
	// Constants
	Filter := stateIn.Filter

	// Algorithm
	return stateIn.In(Filter.V0)
}

/*
IntersectionOfStates
Description:
	Intersects the slices given in the
*/
func IntersectionOfStates(stateSlice1 []ProcrusteanFilterState, otherStateSlices ...[]ProcrusteanFilterState) []ProcrusteanFilterState {
	// Constants
	numOtherSlices := len(otherStateSlices)

	// Algorithm
	var stateSliceIntersection []ProcrusteanFilterState
	for _, tempState := range stateSlice1 {
		tempStateIsInAllOtherSlices := true
		for otherSliceIndex := 0; otherSliceIndex < numOtherSlices; otherSliceIndex++ {
			tempStateIsInAllOtherSlices = tempStateIsInAllOtherSlices && tempState.In(otherStateSlices[otherSliceIndex])
		}
		// Append to stateSliceIntersection
		if tempStateIsInAllOtherSlices {
			stateSliceIntersection = tempState.AppendIfUniqueTo(stateSliceIntersection)
		}
	}

	return stateSliceIntersection
}

/*
UnionOfStates
Description:
	Creates the union between all of the slices given in the input.
*/
func UnionOfStates(stateSlice1 []ProcrusteanFilterState, otherStateSlices ...[]ProcrusteanFilterState) []ProcrusteanFilterState {
	// Constants
	numOtherSlices := len(otherStateSlices)

	// Algorithm
	var stateSliceUnion []ProcrusteanFilterState = stateSlice1
	for otherSliceIndex := 0; otherSliceIndex < numOtherSlices; otherSliceIndex++ {
		for _, tempState := range otherStateSlices[otherSliceIndex] {
			stateSliceUnion = tempState.AppendIfUniqueTo(stateSliceUnion)
		}
	}

	return stateSliceUnion
}

/*
LanguageWithMaxLength()
Description:
	Creates a set (aka slice) of extensions for the given state.
*/
func (stateIn ProcrusteanFilterState) LanguageWithLength(lengthIn int) ([]ExtensionCandidate, error) {
	// Constants
	pf := stateIn.Filter

	// Input Processing
	if lengthIn < 0 {
		return []ExtensionCandidate{}, fmt.Errorf("The input length %v is not positive! Please provide a positive length input!", lengthIn)
	}

	if lengthIn == 0 {
		return []ExtensionCandidate{ExtensionCandidate{s: []string{}}}, nil
	}

	// Algorithm
	var extensionsAt [][]ExtensionCandidate
	var extensionCandidatesAt [][]ExtensionCandidate

	extensionsAt = append(extensionsAt, []ExtensionCandidate{})
	extensionCandidatesAt = append(extensionCandidatesAt, []ExtensionCandidate{})

	var lengthOneSequences []ExtensionCandidate
	for observationIndex := 0; observationIndex < len(pf.Y); observationIndex++ {
		lengthOneSequences = append(lengthOneSequences,
			ExtensionCandidate{s: []string{pf.Y[observationIndex]}, Filter: pf},
		)
	}
	extensionCandidatesAt = append(extensionCandidatesAt, lengthOneSequences)

	s0 := stateIn

	// Use loop to test new candidates and expand the set of candidates
	for T := 1; T <= lengthIn; T++ {
		// Keep track of all extension candidates which are confirmed.
		var confirmedExtensionsAtT []ExtensionCandidate
		for _, extensionCandidate := range extensionCandidatesAt[T] {
			if extensionCandidate.IsExtensionOf(s0) {
				// If this is a true extension of s0, then add it to the list of true
				confirmedExtensionsAtT = extensionCandidate.AppendIfUniqueTo(confirmedExtensionsAtT)
			}
		}
		extensionsAt = append(extensionsAt, confirmedExtensionsAtT)

		// Extend the confirmed extensions
		var nextCandidates []ExtensionCandidate
		for _, tempExtension := range confirmedExtensionsAtT {
			nextCandidates = append(nextCandidates, tempExtension.ExtendByOne()...)
		}
		extensionCandidatesAt = append(extensionCandidatesAt, nextCandidates)
	}

	return extensionsAt[len(extensionsAt)-1], nil

}

/*
LanguageWithMaxLength
Description:
	Returns the language that contains all executions less than or equal to the length in
*/
func (stateIn ProcrusteanFilterState) LanguageWithMaxLength(lengthIn int) ([]ExtensionCandidate, error) {

	// Algorithm
	var LanguageOut []ExtensionCandidate
	for t := 0; t < lengthIn; t++ {
		tempLanguage, err := stateIn.LanguageWithLength(t)
		if err != nil {
			return LanguageOut, err
		}

		LanguageOut = append(LanguageOut, tempLanguage...)
	}

	return LanguageOut, nil
}

/*
S
Description:
	This function returns some subset of the initial states that reach the target state.
	I don't have a proof that this contains all, but I think it is possible.
*/
func (stateIn ProcrusteanFilterState) S() []ExtensionCandidate {
	// Constants
	Filter := stateIn.Filter

	maxLanguageLength := len(Filter.V)

	// Algorithm
	var ExtensionsReachingStateInFromAnInitialState []ExtensionCandidate
	for _, initState := range Filter.V0 {
		// Construct all valid Languages with different lengths and
		// starting from the initial state initState
		maxLanguageForIS, err := initState.LanguageWithMaxLength(maxLanguageLength)
		if err != nil {
			return []ExtensionCandidate{}
		}

		for _, tempExtension := range maxLanguageForIS {
			ReachableSetForIS := initState.ReachesWith(tempExtension.s)
			if stateIn.In(ReachableSetForIS) {
				ExtensionsReachingStateInFromAnInitialState = append(ExtensionsReachingStateInFromAnInitialState, tempExtension)
				break
			}
		}
	}

	return ExtensionsReachingStateInFromAnInitialState
}

/*
CorrespondsWith
Description:

Assumption:
	Assumes that both states have associated filters.
*/
func (stateIn ProcrusteanFilterState) CorrespondsWith(state2 ProcrusteanFilterState) bool {
	// Input Processing
	if (stateIn.Filter == nil) || (state2.Filter == nil) {
		return false
	}

	// Constants

	// Algorithm
	return len(IntersectionOfExtensions(stateIn.S(), state2.S())) != 0
}

/*
K
Description:
	Determines the states from filter F which correspond with the state stateIn from a
	potentially different function.
*/
func (stateIn ProcrusteanFilterState) K(F ProcrusteanFilter) []ProcrusteanFilterState {

	// Constants
	V := F.V

	vi_prime := stateIn
	//F_prime := stateIn.Filter

	// Algorithm
	var correspondingStatesFromF []ProcrusteanFilterState
	for _, v := range V {
		if v.CorrespondsWith(vi_prime) {
			correspondingStatesFromF = append(correspondingStatesFromF, v)
		}
	}

	return correspondingStatesFromF
}

/*
IsCompatibleWith
Description:
	Determines if the two states in the SAME filter are compatible according to
	Definition 11 in the paper. This means that:
	- They agree on the outputs of all their extensions.
*/
func (stateIn ProcrusteanFilterState) IsCompatibleWith(state2 ProcrusteanFilterState) (bool, error) {
	// Input Checking
	if stateIn.Filter != state2.Filter {
		return false, errors.New("The two states come from two different filters!")
	}

	// Constants
	v := stateIn
	w := state2

	Filter := stateIn.Filter
	numFilterStates := len(Filter.V)

	Lv, err := v.LanguageWithMaxLength(2 * numFilterStates)
	if err != nil {
		return false, err
	}

	Lw, err := w.LanguageWithMaxLength(2 * numFilterStates)
	if err != nil {
		return false, err
	}

	// Algorithm
	LIntersection := IntersectionOfExtensions(Lw, Lv)
	if len(LIntersection) == 0 {
		return false, nil
	}

	for _, s := range LIntersection {
		AllvPrime := v.ReachesWith(s.s)
		AllwPrime := w.ReachesWith(s.s)

		for _, vPrime := range AllvPrime {
			for _, wPrime := range AllwPrime {
				// If one of pair of states vPrime and wPrime does not match in output,
				// then these are not compatible.
				tf, err := SliceEquals(Filter.c[vPrime], Filter.c[wPrime])
				if err != nil {
					return false, err
				}
				if !tf {
					return false, nil
				}
			}
		}
	}

	// If all of the matches are good, then return true!
	return true, nil
}

/*
IsMutuallyCompatibleSet
Description:
	Verifies whether or not all states in U are compatible with one another.
*/
func IsMutuallyCompatibleSet(U []ProcrusteanFilterState) bool {
	// Constants

	// Algorithm
	for uIndex, u := range U {
		for uPrimeIndex := uIndex + 1; uPrimeIndex < len(U); uPrimeIndex++ {

			uPrime := U[uPrimeIndex]

			// If u and uPrime are not compatible, then the set is not Mutually Compatible
			if tf, _ := u.IsCompatibleWith(uPrime); !tf {
				return false
			}

		}
	}

	// If the set passes all such tests, then return true
	return true
}

/*
FormZipperConstraint
Description:
	Determines if the sets U and W form a zipper constraint with observation y.
	DEPRECATED. While this still works, it is recommended that you use the ZipperConstraintCandidate object instead.
Assumption:
	Assumes that both slices of states come from the same filter.
*/
func FormZipperConstraint(U []ProcrusteanFilterState, W []ProcrusteanFilterState, y string) bool {
	// Constants

	// Algorithm
	_, err := GetZipperConstraint(U, W, y)
	return err == nil
}

/*
Powerset
Description:
	Creates all possible subsets of the input array of atomic propositions.
*/
func Powerset(setOfStates []ProcrusteanFilterState) [][]ProcrusteanFilterState {
	if len(setOfStates) == 0 {
		return [][]ProcrusteanFilterState{}
	}

	// Constants
	Filter := setOfStates[0].Filter

	// Algorithm
	var AllCombinations [][]ProcrusteanFilterState
	var AllCombinationsAsStrings [][]string

	var AllNames []string
	for _, tempPFS := range setOfStates {
		AllNames = append(AllNames, tempPFS.Name)
	}

	AllCombinationsAsStrings = combinations.All(AllNames)

	for _, tempStringSlice := range AllCombinationsAsStrings {
		AllCombinations = append(AllCombinations, StringSliceToStates(tempStringSlice, Filter))
	}

	AllCombinations = append(AllCombinations, []ProcrusteanFilterState{})

	return AllCombinations
}

/*
StringSliceToStates
Description:
	Converts a slice of strings into a slice containing ProcrusteanFilterState objects with names given by the
	strings and with filter given by F.
*/
func StringSliceToStates(stringSliceIn []string, F *ProcrusteanFilter) []ProcrusteanFilterState {
	// Constants

	// Algorithm
	var pfsSliceOut []ProcrusteanFilterState
	for _, tempName := range stringSliceIn {
		tempPFS := ProcrusteanFilterState{Name: tempName, Filter: F}
		pfsSliceOut = tempPFS.AppendIfUniqueTo(pfsSliceOut)
	}

	return pfsSliceOut
}

/*
IsTerminal
Description:
	Identifies if a given state is terminal or not.
*/
func (stateIn ProcrusteanFilterState) IsTerminal() bool {
	// Constants
	F := stateIn.Filter

	// Algorithm

	// Search for an outgoing edge from this state in tau.
	for _, v := range F.V {
		if len(F.tau[stateIn][v]) != 0 {
			return false
		}
	}

	// If not such transition exists, then this is terminal
	return true

}
