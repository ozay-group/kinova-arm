# System
# Description:
#   A file containing some of the structures needed to represent and manipulate the
#   system object for use in the KAM algorithm.

struct System
    X::Array{String}
    X0::Array{String}
    U::Array{String}
    FAsMatrices::Array{Matrix{Int}}
    Y::Array{String}
    HAsTuples::Array{Tuple}
end

# =========
# Functions
# =========

"""
System(n_X::Integer)
Description:
    An alternative function for defining a new system object.
    This implementation
"""
function System(n_X::Integer)
    # Constants

    # Algorithm
    tempX::Array{String} = []
    for x in range(1,stop=n_X)
        append!(tempX,[string("x",x)])
    end

    return System(tempX,[],[], [], [],[()] )
end


"""
find_state_index_of(name_in::String, system_in::System)
Description:
    Retrieves the index of the state that has name name_in.
"""
function find_state_index_of(name_in::String, system_in::System)
    # Constants

    # algorithm
    for x_index = range(1, stop=length(system_in.X) )
        if system_in.X[x_index] == name_in
            return x_index
        end
    end

    # if the name was not found, then return -1.
    return -1

end

"""
find_input_index_of(name_in::String, system_in::System)
Description:
    Retrieves the index of the state that has name name_in.
"""
function find_input_index_of(name_in::String, system_in::System)
    # Constants

    # algorithm
    for u_index = range(1, stop=length(system_in.U) )
        if system_in.U[u_index] == name_in
            return u_index
        end
    end

    # if the name was not found, then return -1.
    return -1

end

"""
F(x::String, u::String, system_in::System)
Description:
    Attempts to find the set of states that the system will transition to
    from the current state x with input u.
"""
function F(x::String, u::String, system_in::System)
    # Constants
    x_index = find_state_index_of(x,system_in)
    u_index = find_input_index_of(u,system_in)

    # Algorithm
    nextStateIndices = F(x_index,u_index,system_in) # See below for implementation of F for integers.

    nextStatesAsStrings = Array{String}([])
    for nsi_index = 1:length(nextStateIndices)
        push!(nextStatesAsStrings,system_in.X[nextStateIndices[nsi_index]])
    end

    return nextStatesAsStrings

end

"""
F(x_index::Integer, u_index::Integer, system_in::System)
Description:
    Attempts to find the set of states that the system will transition to
    from the current state system_in.X[x_index] with input system_in.U[u_index].
"""
function F(x_index::Integer, u_index::Integer, system_in::System)
    # Constants

    # Algorithm
    binaryVectorOfNextStates = system_in.FAsMatrices[x_index][u_index,:]

    nextStateIndices = Array{Integer}([])
    for next_x_index = 1:length(binaryVectorOfNextStates)
        if binaryVectorOfNextStates[next_x_index] != 0 
            push!(nextStateIndices,next_x_index)
        end
    end

    return nextStateIndices

end
