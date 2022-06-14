# system_test.jl
# Description:
#   This file defines several tests for the System object defined in system.jl.

import Test
import SparseArrays
using Test

include("../src/ModelChecking.jl")

"""
Test 1:
Verify that the empty System can be created using default constructor.
"""

sys1 = TransitionSystem( 
    Array{String}([]),
    Array{String}([]),
    Vector{SparseMatrixCSC{Int,Int}}([]),
    Vector{String}([]),
    Vector{String}([]),
    sparse([1],[1],[0])
    )

@test length(sys1.S) == 0

"""
Test 2:
Verify that the simple TransitionSystem can use find_state_index_of().
"""

sys2 = TransitionSystem(
    ["q0","q1","q3"], 
    [], 
    Vector{SparseMatrixCSC{Int,Int}}([]), 
    [],
    [],
    sparse([],[],[]) )

@test length(sys2.S) == 3
@test find_state_index_of("q3",sys2) == 3
@test find_state_index_of("q5",sys2) == -1

"""
Test 3: (TransitionSystem(n_S))
Tests that the proper number of states are created when using the constructor System(n_S)!
"""
n_S = 10
sys3 = TransitionSystem(n_S)

@test length(sys3.S) == n_S

"""
Test 4: (find_input_index_of())
Verify that the simple TransitionSystem can use find_input_index_of().
"""

sys2 = TransitionSystem(
    ["q0","q1","q3"], 
    ["a0","a1"], 
    Vector{SparseMatrixCSC{Int,Int}}([]), 
    ["q0"], 
    [],
    sparse([],[],[]) )

@test length(sys2.Act) == 2
@test find_action_index_of("a1",sys2) == 2
@test find_action_index_of("a3",sys2) == -1

"""
Section 2: F()
"""

# Test 2a: F using indices
sys2a = TransitionSystem(
    ["q0","q1","q3"], 
    ["o1","o2"],
    Vector{SparseMatrixCSC{Int,Int}}([
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ]), 
    ["q0"], 
    [],
    sparse([],[],[])
)

post_out1 = Post(1,2,sys2a)
@test length(post_out1) == 2
@test 2 in post_out1

# Test 2b: F using strings
sys2b = TransitionSystem(
    ["q0","q1","q3"], 
    ["o1","o2"], 
    [
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ],
    ["q0"], 
    [],
    sparse([],[],[])
)

post_out1 = Post("q0","o2",sys2b)
@test length(post_out1) == 2
@test "q1" in post_out1

# Test2c: F using multiple states as input
sys2c = TransitionSystem(
    ["q0","q1","q2","q3"], 
    ["o1","o2"], 
    [
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[3,4],[1,1]),
        sparse([1,2],[1,2],[1,1])
    ],
    ["q0"], 
    [],
    sparse([],[],[])
)

post_out2 = Post(["q0","q2"],"o2",sys2c)
@test length(post_out2) == 3
@test "q1" in post_out2
@test Set(["q1","q2","q3"]) == Set(post_out2)

"""
Section 5: add_transition!
"""

# Test5a: Adding transition using indices

sys5a = TransitionSystem(
    ["q0","q1","q3"],  
    ["o1","o2"], 
    [
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ],
    ["q0"],
    ["y1","y2","y3"],
    sparse([1,2,3],[2,3,1],[1,1,1])
)
s_ind1 = 1
act_ind1 = 1
s_next_ind1 = 1

@test sys5a.Transitions[s_ind1][act_ind1,s_next_ind1] == 0
add_transition!(sys5a,(s_ind1,act_ind1,s_next_ind1))
@test sys5a.Transitions[s_ind1][act_ind1,s_next_ind1] == 1

# Test5b: add_transition! using name

sys5b = TransitionSystem(
    ["q0","q1","q3"], 
    ["o1","o2"], 
    [
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ],
    ["q0"], 
    ["y1","y2","y3"],
    sparse([1,2,3],[2,3,1],[1,1,1])
)

s_ind1 = 1
act_ind1 = 1
s_next_ind1 = 3

s1 = sys5b.S[s_ind1] # "q0"
act1 = sys5b.Act[act_ind1] # "o1"
s2 = sys5b.S[s_next_ind1] # "q3"

@test sys5b.Transitions[s_ind1][act_ind1,s_next_ind1] == 0
add_transition!(sys5b,(s1,act1,s2))
@test sys5b.Transitions[s_ind1][act_ind1,s_next_ind1] == 1
