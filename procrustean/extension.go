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
