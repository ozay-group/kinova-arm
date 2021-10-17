/*
utilities.go
Description:
	An implementation of the Procrustean filter as described by Yulin Zhang and
	Dylan A. Shell in their 2020 WAFR paper "Cover Combinatorial Filters and
	their Minimization Problem".
*/

package procrustean

import (
	"fmt"

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

/*
Subset
Description:
	Determines if slice1 is a subset of slice2
*/
func SliceSubset(slice1, slice2 interface{}) (bool, error) {

	switch x := slice1.(type) {
	case []string:

		stringSlice1, ok1 := slice1.([]string)
		stringSlice2, ok2 := slice2.([]string)

		if (!ok1) || (!ok2) {
			return false, fmt.Errorf("Error converting slice1? %v ; Error converting slice2? %v", ok1, ok2)
		}

		//Iterate through all strings in stringSliceA and make sure that they are in B.
		for _, tempString := range stringSlice1 {
			if _, inB := oze.FindStringInSlice(tempString, stringSlice2); !inB {
				return false, nil
			}
		}

		// If all elements of slice1 are in slice2 then return true!
		return true, nil

	case []ProcrusteanFilterState:

		pfsSlice1, ok1 := slice1.([]ProcrusteanFilterState)
		pfsSlice2, ok2 := slice2.([]ProcrusteanFilterState)

		if (!ok1) || (!ok2) {
			return false, fmt.Errorf("Error converting slice1? %v ; Error converting slice2? %v", ok1, ok2)
		}

		//Iterate through all strings in stringSliceA and make sure that they are in B.
		for _, tempState := range pfsSlice1 {
			if !tempState.In(pfsSlice2) {
				return false, nil
			}
		}

		// If all elements are in pfsSlice2 then return true!
		return true, nil

	default:
		return false, fmt.Errorf("Unexpected type given to SliceSubset(): %v", x)
	}

}

/*
SliceEquals
Description:
*/
func SliceEquals(slice1, slice2 interface{}) (bool, error) {
	//Determine if both slices are of the same type.
	// if slice1.(type) != slice2.(type) {
	// 	fmt.Println("Types of the two slices are different!")
	// 	return false
	// }

	oneSubsetTwo, err := SliceSubset(slice1, slice2)
	if err != nil {
		return false, fmt.Errorf("There was an issue computing SliceSubset(slice1,slice2): %v", err)
	}

	twoSubsetOne, err := SliceSubset(slice2, slice1)
	if err != nil {
		return false, fmt.Errorf("There was an issue computing SliceSubset(slice2,slice1): %v", err)
	}

	return oneSubsetTwo && twoSubsetOne, nil

}
