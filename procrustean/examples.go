package procrustean

/*
examples.go
Description:
	Some examples that help us easily test the Procrustean filter ideas.
*/

/*
GetPFilter1
Description:
	This should be the same filter from Figure 4.
*/
func GetPFilter1() ProcrusteanFilter {
	pf1, _ := GetProcrusteanFilter(
		[]string{"w0", "w1", "w2", "w3", "w4", "w5", "w6", "w7", "w8", "w9"},
		[]string{"w0"},
		[]string{"a", "b", "c", "e", "f", "g", "h", "o", "p", "q", "r"},
		map[string]map[string][]string{
			"w0": map[string][]string{
				"w1": []string{"o"},
				"w2": []string{"p"},
				"w3": []string{"q"},
				"w4": []string{"r"},
			},
			"w1": map[string][]string{
				"w5": []string{"e"},
				"w9": []string{"b"},
			},
			"w2": map[string][]string{
				"w2": []string{"b"},
				"w8": []string{"f"},
				"w9": []string{"a"},
			},
			"w3": map[string][]string{
				"w3": []string{"c"},
				"w7": []string{"f"},
			},
			"w4": map[string][]string{
				"w4": []string{"a"},
				"w6": []string{"g"},
				"w9": []string{"c"},
			},
			"w5": map[string][]string{
				"w9": []string{"g"},
			},
			"w6": map[string][]string{
				"w4": []string{"g"},
			},
			"w7": map[string][]string{
				"w9": []string{"h"},
			},
			"w8": map[string][]string{
				"w2": []string{"h"},
			},
			"w9": map[string][]string{},
		},
		[]string{"o1", "o2", "o3", "o4", "o5"},
		map[string][]string{
			"w0": []string{"o4"},
			"w1": []string{"o1"},
			"w2": []string{"o1"},
			"w3": []string{"o1"},
			"w4": []string{"o1"},
			"w5": []string{"o2"},
			"w6": []string{"o2"},
			"w7": []string{"o3"},
			"w8": []string{"o3"},
			"w9": []string{"o5"},
		},
	)

	return pf1
}

/*
GetPFilter2
Description:
	This is a simple small filter with four states.
*/
func GetPFilter2() ProcrusteanFilter {
	pf2, _ := GetProcrusteanFilter(
		[]string{"w0", "w1", "w2", "w3"},
		[]string{"w0"},
		[]string{"a", "b"},
		map[string]map[string][]string{
			"w0": map[string][]string{
				"w1": []string{"a"},
			},
			"w1": map[string][]string{
				"w2": []string{"a"},
				"w3": []string{"b"},
			},
			"w2": map[string][]string{
				"w0": []string{"a"},
			},
			"w3": map[string][]string{
				"w0": []string{"b"},
			},
		},
		[]string{"o1", "o2", "o3"},
		map[string][]string{
			"w0": []string{"o1"},
			"w1": []string{"o2"},
			"w2": []string{"o3"},
			"w3": []string{"o3"},
		},
	)

	return pf2
}

/*
GetPFilter3
Description:
	This is a small nondeterministic filter with four states.
*/
func GetPFilter3() ProcrusteanFilter {
	pf1, _ := GetProcrusteanFilter(
		[]string{"w0", "w1", "w2", "w3"},
		[]string{"w0"},
		[]string{"a", "b"},
		map[string]map[string][]string{
			"w0": map[string][]string{
				"w1": []string{"a"},
				"w3": []string{"a"},
			},
			"w1": map[string][]string{
				"w2": []string{"a"},
				"w3": []string{"b"},
			},
			"w2": map[string][]string{
				"w0": []string{"a"},
			},
			"w3": map[string][]string{
				"w0": []string{"b"},
			},
		},
		[]string{"o1", "o2", "o3"},
		map[string][]string{
			"w0": []string{"o1"},
			"w1": []string{"o2"},
			"w2": []string{"o3"},
			"w3": []string{"o3"},
		},
	)

	return pf1
}

/*
GetPFilter4
Description:
	This is a small nondeterministic filter with four states.
	It is modified so that leaving the similar states w2 or w3 is done with the same symbol
	which should make those two states be compatible.
*/
func GetPFilter4() ProcrusteanFilter {
	pf1, _ := GetProcrusteanFilter(
		[]string{"w0", "w1", "w2", "w3"},
		[]string{"w0"},
		[]string{"a", "b"},
		map[string]map[string][]string{
			"w0": map[string][]string{
				"w1": []string{"a"},
			},
			"w1": map[string][]string{
				"w2": []string{"a"},
				"w3": []string{"b"},
			},
			"w2": map[string][]string{
				"w0": []string{"a"},
			},
			"w3": map[string][]string{
				"w0": []string{"a"},
			},
		},
		[]string{"o1", "o2", "o3"},
		map[string][]string{
			"w0": []string{"o1"},
			"w1": []string{"o2"},
			"w2": []string{"o3"},
			"w3": []string{"o3"},
		},
	)

	return pf1
}

/*
GetPFilter5
Description:
	This is the filter from Figure 5 in the paper.
*/
func GetPFilter5() ProcrusteanFilter {
	pf1, _ := GetProcrusteanFilter(
		[]string{"w0", "w1", "w2", "w3", "w4", "w5", "w6", "w7", "w8", "w9"},
		[]string{"w0"},
		[]string{"a", "b", "c", "d"},
		map[string]map[string][]string{
			"w0": map[string][]string{
				"w1": []string{"a"},
				"w2": []string{"b"},
				"w3": []string{"c"},
				"w4": []string{"d"},
			},
			"w1": map[string][]string{
				"w5": []string{"a"},
			},
			"w2": map[string][]string{
				"w6": []string{"a"},
			},
			"w3": map[string][]string{
				"w6": []string{"b"},
			},
			"w4": map[string][]string{
				"w7": []string{"b"},
			},
			"w5": map[string][]string{
				"w8": []string{"c"},
			},
			"w6": map[string][]string{},
			"w7": map[string][]string{
				"w9": []string{"c"},
			},
			"w8": map[string][]string{},
			"w9": map[string][]string{},
		},
		[]string{"o0", "o1", "o2", "o3", "o4", "o5"},
		map[string][]string{
			"w0": []string{"o0"},
			"w1": []string{"o1"},
			"w2": []string{"o1"},
			"w3": []string{"o2"},
			"w4": []string{"o2"},
			"w5": []string{"o3"},
			"w6": []string{"o3"},
			"w7": []string{"o3"},
			"w8": []string{"o4"},
			"w9": []string{"o5"},
		},
	)

	return pf1
}

/*
GetPFilter6
Description:
	This is the filter from Figure 5 in the paper.
	Modified this filter to make it work for the claims of compatible states.
*/
func GetPFilter6() ProcrusteanFilter {
	pf1, _ := GetProcrusteanFilter(
		[]string{"w0", "w1", "w2", "w3", "w4", "w5", "w6", "w7", "w8", "w9"},
		[]string{"w0"},
		[]string{"a", "b", "c", "d"},
		map[string]map[string][]string{
			"w0": map[string][]string{
				"w1": []string{"a"},
				"w2": []string{"b"},
				"w3": []string{"c"},
				"w4": []string{"d"},
			},
			"w1": map[string][]string{
				"w5": []string{"a"},
			},
			"w2": map[string][]string{
				"w6": []string{"a"},
			},
			"w3": map[string][]string{
				"w6": []string{"b"},
			},
			"w4": map[string][]string{
				"w7": []string{"b"},
			},
			"w5": map[string][]string{
				"w8": []string{"c"},
			},
			"w6": map[string][]string{
				"w9": []string{"c"},
			},
			"w7": map[string][]string{
				"w9": []string{"c"},
			},
			"w8": map[string][]string{},
			"w9": map[string][]string{},
		},
		[]string{"o0", "o1", "o2", "o3", "o4", "o5"},
		map[string][]string{
			"w0": []string{"o0"},
			"w1": []string{"o1"},
			"w2": []string{"o1"},
			"w3": []string{"o2"},
			"w4": []string{"o2"},
			"w5": []string{"o3"},
			"w6": []string{"o3"},
			"w7": []string{"o3"},
			"w8": []string{"o4"},
			"w9": []string{"o5"},
		},
	)

	return pf1
}
