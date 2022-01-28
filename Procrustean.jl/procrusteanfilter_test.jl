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

"""
Test 3:
Verify that the simple ProcrusteanFilter can use observation_name_to_index.
"""

pf2 = ProcrusteanFilter( ["q0","q1","q3"], ["q0"], ["o1","o2"], [], [],[[]] )

@test length(pf2.Y) == 2
@test observation_name_to_index(pf2,"o1") == 1
@test observation_name_to_index(pf2,"o5") == -1

"""
Test 4:
Verify that the simple ProcrusteanFilter can use Post(pf,v,y).
"""

pf3 = ProcrusteanFilter( 
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [ [0 0; 0 1; 0 1], [0 0; 1 1; 0 1] , [1 0; 0 1; 0 0] ],
    [],[[]] )

@test length(pf3.Transitions) == length(pf3.V)
@test Post(pf3,"q0","o1") == []
@test Post(pf3,"q0","o2") == ["q1","q3"]
@test Post(pf3,"q3","o2") == ["q1"]

"""
Test 5:
Verify that the simple ProcrusteanFilter can use Post(pf,v,y).
"""

pf4 = ProcrusteanFilter( 
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [ [0 0; 0 1; 0 1], [0 0; 1 1; 0 1] , [0 0; 1 1; 0 0] ],
    [],[[]] )

@test length(pf4.Transitions) == length(pf3.V)
@test Post(pf4,"q0") == ["q1","q3"]
@test Post(pf4,"q3") == ["q1"]

"""
Test 6:
Verify that the simple ProcrusteanFilter can use add_transition!(pf,v,y,v_prime).
"""

pf5 = ProcrusteanFilter( 
    ["q0","q1","q3"], 
    ["q0"], 
    ["o1","o2"], 
    [ [0 0; 0 1; 0 1], [0 0; 1 1; 0 1] , [0 0; 1 1; 0 0] ],
    [],[[]] )

@test pf5.Transitions[2][1,1] == 0

add_transition!(pf5,"q1","o1","q0")
@test pf5.Transitions[2][1,1] == 1 # The new transition should now be present in the matrices.
