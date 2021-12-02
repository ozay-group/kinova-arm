"""
procrusteanfilter_test.jl
Description:

"""


import Test
using Test

include("procrusteanfilter.jl")

"""
Test 1:
Verify that the empty ProcrusteanFilter can be created using default constructor.
"""

pf0 = ProcrusteanFilter( 
    Vector{String}(),
    Vector{String}(),
    Vector{String}(),
    AbstractVector{AbstractMatrix{Int}}([]),
    Vector{String}([]),
    Vector{Vector{String}}([[]])
    )

@test length(pf0.V) == 0

"""
Test 2:
Verify that the simple ProcrusteanFilter can use state_name_to_index.
"""

pf1 = ProcrusteanFilter( ["q0","q1","q3"], [], [], [], [],[[]] )

@test length(pf1.V) == 3
@test state_name_to_index(pf1,"q3") == 3
@test state_name_to_index(pf1,"q5") == -1