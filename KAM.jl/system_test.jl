# system_test.jl
# Description:
#   This file defines several tests for the System object defined in system.jl.

import Test
import SparseArrays
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
    Vector{SparseMatrixCSC{Int,Int}}([]),
    Vector{String}([]),
    sparse([],[],[])
    )

@test length(sys1.X) == 0

"""
Test 2:
Verify that the simple System can use find_state_index_of().
"""

sys2 = System(
    ["q0","q1","q3"], 
    [], 
    [], 
    Vector{SparseMatrixCSC{Int,Int}}([]), 
    [],
    sparse([],[],[]) )

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

sys2 = System(
    ["q0","q1","q3"], 
    ["q0"], 
    ["a0","a1"], 
    Vector{SparseMatrixCSC{Int,Int}}([]), 
    [],
    sparse([],[],[]) )

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
    Vector{SparseMatrixCSC{Int,Int}}([
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ]), 
    [],
    sparse([],[],[])
)

f_out1 = F(1,2,sys2a)
@test length(f_out1) == 2
@test 2 in f_out1

# Test 2b: F using strings
sys2b = System(
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ],
    [],
    sparse([],[],[])
)

f_out1 = F("q0","o2",sys2a)
@test length(f_out1) == 2
@test "q1" in f_out1

"""
Section 3: H()
"""

# Test 3a: H using indices
sys3a = System(
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [ 
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ],
    ["y1","y2","y3"],
    sparse([1,2,3],[2,3,1],[1,1,1])
)

@test H(1,sys3a) == [2]

# Test 3b: H using name
sys3a = System(
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ],
    ["y1","y2","y3"],
    sparse([1,2,3],[2,3,1],[1,1,1])
)

@test H("q0",sys3a) == ["y2"]

"""
Section 4: HInverse
"""

# Test4a: HInverse using index

sys4a = System(
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ],
    ["y1","y2","y3"],
    sparse([1,2,3],[2,3,1],[1,1,1])
)

tempHInverse = HInverse(1,sys4a)
@test length(tempHInverse) == 1
@test tempHInverse[1] == 3

# Test4b: HInverse using name

sys4b = System(
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ],
    ["y1","y2","y3"],
    sparse([1,2,3],[2,3,1],[1,1,1])
)

tempHInverse = HInverse("y1",sys4a)
@test length(tempHInverse) == 1
@test tempHInverse[1] == "q3"

"""
Section 5: add_transition!
"""

# Test5a: Adding transition using indices

sys5a = System(
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ],
    ["y1","y2","y3"],
    sparse([1,2,3],[2,3,1],[1,1,1])
)
x_ind1 = 1
u_ind1 = 1
x_next_ind1 = 1

@test sys5a.FAsMatrices[x_ind1][u_ind1,x_next_ind1] == 0
add_transition!(sys5a,(x_ind1,u_ind1,x_next_ind1))
@test sys5a.FAsMatrices[x_ind1][u_ind1,x_next_ind1] == 1

# Test5b: add_transition! using name

sys5b = System(
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [
        sparse([2,2],[2,3],[1,1]),
        sparse([1,2,2],[2,2,3],[1,1,1]),
        sparse([1,2],[1,2],[1,1])
    ],
    ["y1","y2","y3"],
    sparse([1,2,3],[2,3,1],[1,1,1])
)

x_ind1 = 1
u_ind1 = 1
x_next_ind1 = 3

x1 = sys5b.X[x_ind1] # "q0"
u1 = sys5b.U[u_ind1] # "o1"
x2 = sys5b.X[x_next_ind1] # "q3"

@test sys5b.FAsMatrices[x_ind1][u_ind1,x_next_ind1] == 0
add_transition!(sys5b,(x1,u1,x2))
@test sys5b.FAsMatrices[x_ind1][u_ind1,x_next_ind1] == 1

