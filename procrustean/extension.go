/*
procrusteanfilter.go
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
)

/*
Type Definition
*/
type ExtensionCandidate struct {
	s      []string
	Filter *ProcrusteanFilter
}

/*
 * Member Functions
 */

/*
Check
Description:
	Determines whether or not the Extension candidate is valid according to the filter provided by Filter.
*/
func (ec ExtensionCandidate) Check() error {
	// Input Checking
	if ec.Filter == nil {
		return errors.New("The ExtensionCandidate has an invalid pointer for its Procrustean Filter.")
	}

	for _, observation := range ec.s {
		// Check to see if observation is in the slice
		if tempInd, tf := oze.FindStringInSlice(observation, ec.Filter.Y); !tf {
			fmt.Sprintf("%v:%v", tempInd, tf)
			return fmt.Errorf("The observation \"%v\" from the extension candidate was not defined in the targeted P-Filter.", observation)
		}
	}

	//If all of these checks are satisfied, then return nil
	return nil
}

/*
IsExtensionOf
Description:
	This is true if and only if the candidate:
	- Reaches a non-empty set of states from the initial state
*/
func (ec ExtensionCandidate) IsExtensionOf(pfs0 ProcrusteanFilterState) bool {
	// Check to see if candidate satisfies basic assumptions
	err := ec.Check()
	if err != nil {
		return false
	}

	// The empty string is always a valid extension
	if len(ec.s) == 0 {
		return true
	}

	// Create the reached states from the candidate
	R0 := pfs0.ReachesWith(ec.s)
	return len(R0) != 0

}

/*
ExtendByOne
Description:
	This function defines all strings (extension candidates) that are formed by adding a new symbol to the current extension
	If the
*/
func (ec ExtensionCandidate) ExtendByOne() []ExtensionCandidate {
	// Check to see if candidate satisfies basic assumptions
	err := ec.Check()
	if err != nil {
		return []ExtensionCandidate{} // If check fails, return empty set.
	}

	// Constants
	Filter := ec.Filter

	// Algorithm
	var extendedVersionsOfEC []ExtensionCandidate
	for _, observation := range Filter.Y {
		extendedVersionsOfEC = append(extendedVersionsOfEC,
			ExtensionCandidate{s: append(ec.s, observation), Filter: Filter},
		)
	}

	return extendedVersionsOfEC
}

/*
Length
Description:
	Returns the lenght of the observation sequence (s) of the given ExtensionCandidate.
*/
func (ec ExtensionCandidate) Length() int {
	return len(ec.s)
}

/*
Equals
*/
func (ec ExtensionCandidate) Equals(ec2 ExtensionCandidate) bool {
	// Constants

	// Input PRocessing
	if ec.Length() != ec2.Length() {
		return false // if the two candidates have different lengths, then they must clearly be different.
	}

	// Check each element in sequence
	for observationIndex := 0; observationIndex < ec.Length(); observationIndex++ {
		if ec.s[observationIndex] != ec2.s[observationIndex] {
			return false
		}
	}

	return true
}

/*
Find
Description:
	Finds the index in ecSlice that matches the input extension candidate.
*/
func (ec ExtensionCandidate) Find(ecSlice []ExtensionCandidate) int {
	// Constants

	// Algorithm
	matchingIndex := -1

	for ecIndex, tempEC := range ecSlice {
		if ec.Equals(tempEC) {
			matchingIndex = ecIndex
		}
	}

	return matchingIndex
}

/*
In
Description:
	Returns true if the extension candidate object is in the slice ecSlice.
*/
func (ec ExtensionCandidate) In(ecSlice []ExtensionCandidate) bool {
	return ec.Find(ecSlice) != -1
}

/*
AppendIfUniqueTo
Description:

*/
func (ec ExtensionCandidate) AppendIfUniqueTo(ecSlice []ExtensionCandidate) []ExtensionCandidate {
	// Constants

	// Algorithm

	if ec.In(ecSlice) {
		return ecSlice
	}

	return append(ecSlice, ec)
}
