# System
# Description:
#   A file containing some of the structures needed to represent and manipulate the
#   system object for use in the KAM algorithm.

using SparseArrays

# =======
# Objects
# =======

struct System
    X::Vector{String}
    X0::Vector{String}
    U::Vector{String}
    FAsMatrices::Vector{SparseMatrixCSC{Int,Int}}
    Y::Vector{String}
    HAsMatrix::SparseMatrixCSC{Int,Int}
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

    return System(tempX,[],[], Vector{SparseMatrixCSC{Int,Int}}([]), [],sparse([],[],[]) )
end

"""
System(n_X::Integer,n_U::Integer,n_Y::Integer)
"""
function System(n_X::Integer,n_U::Integer,n_Y::Integer)
    # Constants

    # Algorithm

    #x
    tempX::Vector{String} = []
    for x_index in range(1,stop=n_X)
        append!(tempX,[string("x",x_index)])
    end

    # Create n_U strings for U
    tempU::Vector{String} = []
    for u_index in range(1,stop=n_U)
        append!(tempU,[string("u_",u_index)])
    end

    # Create n_Y strings for Y
    tempY::Vector{String} = []
    for y_index in range(1,stop=n_Y)
        append!(tempY,[string("y_",y_index)])
    end

    return System(tempX,[],tempU, Array{Int}(undef,0,0,0), tempY,Matrix{Int}(undef,0,0) )

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
    Retrieves the index of the INPUT that has name name_in.
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
find_output_index_of(name_in::String, system_in::System)
Description:
    Retrieves the index of the OUTPUT that has name name_in.
"""
function find_output_index_of(name_in::String, system_in::System)
    # Constants

    # algorithm
    for y_index = range(1, stop=length(system_in.Y) )
        if system_in.Y[y_index] == name_in
            return y_index
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
    F_x = system_in.FAsMatrices[x_index]
    if F_x == []
        throw(DomainError("No transitions are defined for the system from the state "+string(x_index)+"."))
    end

    tempI, tempJ, tempV = findnz(F_x)
    matching_indices = findall( tempI .== u_index )

    return tempJ[ matching_indices ]

end

"""
H(x::String,system_in::System)
Description:
    When an input state is given, this function produces the output from the set of strings Y.
"""
function H(x::String,system_in::System)
    # Constants

    # Algorithm
    x_index = find_state_index_of(x,system_in)

    return system_in.Y[ H( x_index , system_in ) ]
end

"""
H(x_index::Integer,system_in::System)
Description:
    This function receives as input an index for the state of the system 
    (i.e. x_index such that X[x_index] is the state in question), and returns
    the index of the output that matches it.
"""
function H(x_index::Integer,system_in::System)
    # Constants

    # Algorithm
    tempXIndices, tempYIndices, tempV = findnz(system_in.HAsMatrix)
    matching_indices = findall( tempXIndices .== x_index )

    return tempYIndices[ matching_indices ]
end


"""
HInverse(y::String,system_in::System)
Description:
    Returns the names of all states that have output y
"""
function HInverse(y::String,system_in::System)
    # Constants

    # Algorithm
    y_index = find_output_index_of(y,system_in)
    HInverse_as_indices = HInverse(y_index,system_in)

    matching_states = Array{String}([])
    for temp_index in HInverse_as_indices
        push!(matching_states,system_in.X[temp_index])
    end

    return matching_states

end

function HInverse(y_index::Integer,system_in::System)
    # Constants
    num_states = length(system_in.X)

    # Algorithm    
    tempXIndices, tempYIndices, tempV = findnz(system_in.HAsMatrix)
    matching_indices = findall( tempYIndices .== y_index )

    # Return matching states
    return tempXIndices[matching_indices]
end

"""
add_transition!(system_in::System,transition_in::Tuple{Int,Int,Int})
Description:
    Adds a transition to the system system_in according to the tuple of indices tuple_in.
        tuple_in = (x_in,u_in,x_next_in)
"""
function add_transition!(system_in::System,transition_in::Tuple{Int,Int,Int})
    # Constants
    x_in = transition_in[1]
    u_in = transition_in[2]
    x_next_in = transition_in[3]

    # Checking inputs
    check_x(x_in,system_in)
    check_u(u_in,system_in)
    check_x(x_next_in,system_in)
    
    # Algorithm
    system_in.FAsMatrices[x_in][u_in,x_next_in] = 1
    
end

"""
add_transition!(system_in::System,transition_in::Tuple{String,String,String})
Description:
    Adds a transition to the system system_in according to the tuple of names tuple_in.
        tuple_in = (x_in,u_in,x_next_in)
"""
function add_transition!(system_in::System,transition_in::Tuple{String,String,String})
    # Constants
    x_in = transition_in[1]
    u_in = transition_in[2]
    x_next_in = transition_in[3]

    # Checking inputs
    
    x_index = find_state_index_of(x_in,system_in)
    u_index = find_input_index_of(u_in,system_in)
    x_next_index = find_state_index_of(x_next_in,system_in)

    # Algorithm
    add_transition!(system_in,( x_index , u_index , x_next_index ))
    
end

"""
check_x(x_in::Integer,system_in::System)
Description:
    Checks to make sure that a possible state index is actually in the bounds of the s
"""
function check_x(x_in::Integer,system_in::System)
    # Constants
    n_X = length(system_in.X)

    # Algorithm
    if (1 > x_in) || (n_X < x_in)
        throw(DomainError("The input transition references a state " * string(x_in) * " which is not in the state space!"))
    end

    return
end

"""
check_x(x_in::String,system_in::System)
Description:
    Checks to make sure that a possible state index is actually in the bounds of the s
"""
function check_x(x_in::String,system_in::System)
    # Constants

    # Algorithm
    if !(x_in in system_in.X)
        throw(DomainError("The input transition references a state " * string(x_in) * " which is not in the state space!"))
    end

    return
end

"""
check_u(u_index_in::Integer,system_in::System)
Description:
    Checks to make sure that a possible state index is actually in the bounds of the s
"""
function check_u(u_index_in::Integer,system_in::System)
    # Constants
    n_U = length(system_in.U)

    # Algorithm
    if (1 > u_index_in) || (n_X < u_index_in)
        throw(DomainError("The input transition references a state " * string(u_index_in) * " which is not in the input space!"))
    end

    return
end

"""
check_u(u_in::String,system_in::System)
Description:
    Checks to make sure that a possible state index is actually in the bounds of the s
"""
function check_u(u_in::String,system_in::System)
    # Constants

    # Algorithm
    if !(u_in in system_in.U)
        throw(DomainError("The input transition references a state " * string(u_in) * " which is not in the input space!"))
    end

    return
end