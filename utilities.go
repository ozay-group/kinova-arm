/*
utilities.go
Description:
	All the files which are useful for manipulating objects in the OzayGroupExploration
	reppository but which are not attached to a specific type/class.
*/
package OzayGroupExploration

/*
FindStringInSlice
Description:
	Finds if the string stringIn is in the slice stringSliceIn.
*/
func FindStringInSlice(stringIn string, stringSliceIn []string) (int, bool) {
	// Initialize search parameters
	stringIndex := -1

	// Search
	for tempIndex, tempString := range stringSliceIn {
		if tempString == stringIn {
			stringIndex = tempIndex
		}
	}

	// Return result
	return stringIndex, stringIndex >= 0
}

/*
AppendStringToSliceIfUnique
Description:
	Appends the string to the slice stringSliceIn only if stringIn is not already in there.
*/
func AppendStringToSliceIfUnique(stringIn string, stringSliceIn []string) []string {
	// Constants

	// Algorithm
	_, tf := FindStringInSlice(stringIn, stringSliceIn)
	if tf {
		return stringSliceIn
	}

	// If not, then append string
	return append(stringSliceIn, stringIn)

}
