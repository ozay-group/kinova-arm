/*
cliquecover.go
Description:
	An implementation of the Clique Cover and its functions as described by Yulin Zhang and
	Dylan A. Shell in their 2020 WAFR paper "Cover Combinatorial Filters and
	their Minimization Problem".
*/
package procrustean

import "fmt"

// Type Definitions
// ================

type CliqueCover struct {
	K [][]ProcrusteanFilterState
}

// Methods
// =======

func (cc CliqueCover) SatisfiesZipperConstraints(zip []ZipperConstraint) bool {
	// Constants

	// Algorithm
	for _, constraint := range zip {
		// Determine if for this zipper constraint, there exists a clique K_s in cc
		// such that U_i \subseteq K_s then there exists another clique K_t in cc such that W_i is a subset of K_t

		U := constraint.U
		W := constraint.W
		//y := constraint.y

		for s, K_s := range cc.K {
			// Determine if U_i \subseteq K_s
			if tf, _ := SliceSubset(U, K_s); tf {
				// If so, then there exists another clique K_t in cc such that W_i is a subset of K_t
				K_t_exists := false
				for t, K_t := range cc.K {
					// Skip the set K_s
					if s == t {
						continue
					}

					tf2, _ := SliceSubset(W, K_t)
					K_t_exists = K_t_exists || tf2

				}
				// If K_t does not exist, then return false.
				if !K_t_exists {
					return false
				}
			}
		}
	}

	// If this passes all tests, then return false
	return true
}

/*
ToInducedFilter
Description:

	From Definition 19.
Assumptions:
	Assumes that the CliqueCover was made on a SINGLE ProcrusteanFilter.
*/
func (cc CliqueCover) ToInducedFilter() (ProcrusteanFilter, error) {
	// Constants
	F := cc.K[0][0].Filter
	cg := F.ToCompatibilityGraph()

	// Algorithm
	var F_prime ProcrusteanFilter

	// check that clique cover satisfies the filter F's zipper constraints
	if !cc.SatisfiesZipperConstraints(cg.GetAllZipperConstraints()) {
		return ProcrusteanFilter{}, fmt.Errorf("The proposed clique cover does not satisfy all zipper constraints from the compatibility graph!")
	}

	// Create the states
	for _, K_v_prime := range cc.K {
		v_prime_name := "("
		for _, v := range K_v_prime {
			v_prime_name = fmt.Sprintf("%v%v", v_prime_name, v.Name)
		}
		v_prime_name = fmt.Sprintf("%v)", v_prime_name)

		tempState := ProcrusteanFilterState{
			Name:   v_prime_name,
			Filter: &F_prime,
		}

		F_prime.V = append(F_prime.V, tempState)
	}

	// Create the initial states
	for _, v0 := range F.V0 {
		for v_prime_index, K_v_prime := range cc.K {
			v_prime := F_prime.V[v_prime_index]

			// If an initial state is included in the clique K_v_prime
			// then add v_prime (the state) to the set of initial states.
			if v0.In(K_v_prime) {
				F_prime.V0 = v_prime.AppendIfUniqueTo(F_prime.V0)
			}
		}
	}

	// Create the outputs as the set of common outputs for all states in K_v_prime
	outputMap := make(map[ProcrusteanFilterState][]string)
	for v_prime_index, K_v_prime := range cc.K {
		// Collect the COMMON outputs for each element v in K_v_prime
		commonOutputs := F.c[K_v_prime[0]]
		for _, v := range K_v_prime {
			commonOutputs = IntersectionOfStringSlices(commonOutputs, F.c[v])
		}
		// Create output
		v_prime := F_prime.V[v_prime_index]
		outputMap[v_prime] = commonOutputs
	}
	F_prime.c = outputMap

	// Create the transition map
	transitionMap := make(map[ProcrusteanFilterState]map[ProcrusteanFilterState][]string)
	for v_prime_index, K_v_prime := range cc.K {
		v_prime := F_prime.V[v_prime_index]
		transitionsFromVPrime := make(map[ProcrusteanFilterState][]string)

		// Observe if v_prime transitions to any states in w
		for w_prime_index, K_w_prime := range cc.K {
			w_prime := F_prime.V[w_prime_index]

			// Enumerate all states in v_prime and w_prime
			for _, v := range K_v_prime {
				for _, w := range K_w_prime {
					tauForvw := F.tau[v][w]
					transitionsFromVPrime[w_prime] = UnionOfStringSlices(tauForvw, transitionsFromVPrime[w_prime])
				}
			}
		}
		transitionMap[v_prime] = transitionsFromVPrime
	}
	F_prime.tau = transitionMap

	// Return final filter
	return F_prime, nil

}
