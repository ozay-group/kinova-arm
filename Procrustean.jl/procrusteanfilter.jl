"""
procrusteanfilter.jl
"""

struct ProcrusteanFilterState
    Name::String
end

struct ProcrusteanFilter
    V::AbstractVector{String}
    V0::AbstractVector{String}
    Y::AbstractVector{String}
    Transitions::AbstractVector{AbstractMatrix{Int}}
    C::AbstractVector{String}
    c::AbstractVector{AbstractVector{String}}
    # Additional Inputs
end

"""
GetProcrusteanFilter
Description:

"""
function GetProcrusteanFilter( names::AbstractVector{String} , initialNames::AbstractVector{String} )
    # Constants

    # Algorithm

end

"""
state_name_to_index
Description:

"""
function state_name_to_index( pf::ProcrusteanFilter , q::String )
    # Constants
    num_states = length(pf.V)

    # Algorithm
    for v_index in 1:num_states
        temp_v = pf.V[v_index]
        if temp_v == q
            return v_index
        end
    end

    # If this state wasn't found, then return -1.
    return -1
        
end

