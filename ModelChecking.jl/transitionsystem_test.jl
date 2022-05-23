"""
transitionsystem_test.jl
Description:
    Tests the files from transitionsystem.jl
"""

import Test
using Test

include("transitionsystem.jl")

# Define a Simple TransitionSystemState
ts0 = TransitionSystem( 
    Array{String}(["s1","s2","s3","s4"]),
    Vector{String}(["a1"]), 
    Array{Matrix{Int}}(
        [ [1  2 ; 3 4],  [1  2 ; 3 4] ]
    ),
    Array{String}(["s1","s3"]),
    Vector{String}(["A"]),
    Vector{Vector{String}}([["A","B"],["A"]]),
    Vector{String}(["Y1","Y2"])
)

@test length(ts0.Act) == 1

"""
Test Set 2
"""
ts1 = GetTransitionSystem1(4,2,
    Array{Matrix{Int}}(
        [ [1 2 ; 3 4] , [1 2 ; 3 4] ]
    ),
    Array{Int}([1,3]),
    3,
    Vector{Vector{Int}}([[1,2],[2],[1,3],[3]])
)

print(ts1)