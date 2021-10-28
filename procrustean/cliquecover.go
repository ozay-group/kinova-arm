/*
cliquecover.go
Description:
	An implementation of the Clique Cover and its functions as described by Yulin Zhang and
	Dylan A. Shell in their 2020 WAFR paper "Cover Combinatorial Filters and
	their Minimization Problem".
*/
package procrustean

// Type Definitions
// ================

type CliqueCover struct {
	K [][]ProcrusteanFilterState
}

// Methods
// =======

func (cc CliqueCover) SatisfiesZipperConstraints(zip []ZipperConstraintCandidate) bool {
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
