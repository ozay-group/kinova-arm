/*
utilities.go
Description:
	An implementation of the Procrustean filter as described by Yulin Zhang and
	Dylan A. Shell in their 2020 WAFR paper "Cover Combinatorial Filters and
	their Minimization Problem".
*/

package procrustean

import (
	oze "github.com/kwesiRutledge/OzayGroupExploration"
)

/*
IntersectionOfStringSlices
Description:
	Finds if the string stringIn is in the slice stringSliceIn.
*/
func IntersectionOfStringSlices(stringslice1 []string, otherStringSlices ...[]string) []string {
	// Constants
	numOtherSlices := len(otherStringSlices)

	// Input Processing

	// If there are no other slices to union, then return the value of just the first string slice
	if numOtherSlices == 0 {
		return stringslice1
	}

	// Determine which element from stringSlice1 is in ALL other slices in otherStringSlices
	var intersectionOfStringSlices []string
	for _, tempString := range stringslice1 {
		tempStringInAllStrings := true
		for osSliceIndex := 0; osSliceIndex < numOtherSlices; osSliceIndex++ {
			if _, tf := oze.FindStringInSlice(tempString, otherStringSlices[osSliceIndex]); !tf {
				tempStringInAllStrings = false
			}
		}

		// If the string is in all strings, then add it to the intersection
		if tempStringInAllStrings {
			intersectionOfStringSlices = append(intersectionOfStringSlices, tempString)
		}
	}

	// Return result
	return intersectionOfStringSlices
}
