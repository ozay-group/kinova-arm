# system_test.jl
# Description:
#   This file defines several tests for the System object defined in system.jl.

import Test
using Test

include("system.jl")

"""
Test 1:
Verify that the empty System can be created using default constructor.
"""

sys1 = System( 
    Array{String}([]),
    Array{String}([]),
    Array{String}([]),
    AbstractVector{AbstractMatrix{Int}}([]),
    Vector{String}([]),
    Vector{Tuple{String}}()
    )

@test length(sys1.X) == 0

"""
Test 2:
Verify that the simple System can use find_state_index_of().
"""

sys2 = System(["q0","q1","q3"], [], [], [], [],[()] )

@test length(sys2.X) == 3
@test find_state_index_of("q3",sys2) == 3
@test find_state_index_of("q5",sys2) == -1

"""
Test 3: (System(n_X))
Tests that the proper number of states are created when using the constructor System(n_X)!
"""
n_X = 10
sys3 = System(n_X)

@test length(sys3.X) == n_X

"""
Test 4: (find_input_index_of())
Verify that the simple System can use find_input_index_of().
"""

sys2 = System(["q0","q1","q3"], ["q0"], ["a0","a1"], [], [],[()] )

@test length(sys2.U) == 2
@test find_input_index_of("a1",sys2) == 2
@test find_input_index_of("a3",sys2) == -1

"""
Section 2: F()
"""

# Test 2a: F using indices
sys2a = System(
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [ [0 0 0; 0 1 1], [0 1 0; 0 1 1] , [1 0 0; 0 1 0] ],
    [],[()]
)

f_out1 = F(1,2,sys2a)
@test length(f_out1) == 2
@test 2 in f_out1

# Test 2b: F using strings
sys2b = System(
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [ [0 0 0; 0 1 1], [0 1 0; 0 1 1] , [1 0 0; 0 1 0] ],
    [],[()]
)

f_out1 = F("q0","o2",sys2a)
@test length(f_out1) == 2
@test "q1" in f_out1
