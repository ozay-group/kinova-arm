package procrustean

/*
extension_test.go
Description:
	Tests some of the member functions of the extension.go file.
*/

import (
	"fmt"
	"testing"
)

/*
TestExtension_Check1
Description:
	Supposed to catch when the Filter argument of an ExtensionCandidate object is not defined.
*/
func TestExtension_Check1(t *testing.T) {
	// Constants
	pf1 := GetPFilter1()

	s := pf1.Y[1:3]

	// Create an extension
	ec1 := ExtensionCandidate{s: s, Filter: nil}

	err := ec1.Check()
	if err == nil {
		t.Errorf("Check() did not find that there was an error in the extension candidate value.")
	} else {
		if err.Error() != "The ExtensionCandidate has an invalid pointer for its Procrustean Filter." {
			t.Errorf("Check returned an unexpected error: %v", err)
		}
	}

}

/*
TestExtension_Check2
Description:
	Supposed to catch when the sequence of observations in an ExtensionCandidate contains an unexpected observation.
*/
func TestExtension_Check2(t *testing.T) {
	// Constants
	pf1 := GetPFilter1()

	s := []string{pf1.Y[1], "Yopper"}

	// Create an extension
	ec1 := ExtensionCandidate{s: s, Filter: &pf1}

	err := ec1.Check()
	if err == nil {
		t.Errorf("Check() did not find that there was an error in the extension candidate value.")
	} else {
		if err.Error() != fmt.Sprintf("The observation \"%v\" from the extension candidate was not defined in the targeted P-Filter.", "Yopper") {
			t.Errorf("Check returned an unexpected error: %v", err)
		}
	}

}

/*
TestExtension_Check3
Description:
	Supposed to correctly identify that the candidate is valid by itself.
*/
func TestExtension_Check3(t *testing.T) {
	// Constants
	pf1 := GetPFilter1()

	s := pf1.Y[1:3]

	// Create an extension
	ec1 := ExtensionCandidate{s: s, Filter: &pf1}

	err := ec1.Check()
	if err != nil {
		t.Errorf("Check() found an error when there should not have.")
	}

}
