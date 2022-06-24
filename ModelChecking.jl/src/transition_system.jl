# System
# Description:
#   A file containing some of the structures needed to represent and manipulate the
#   system object for use in the KAM algorithm.

using SparseArrays, Graphs, GraphPlot

# =======
# Objects
# =======

struct TransitionSystem
    S::Vector{String}
    Act::Vector{String}
    Transitions::Vector{SparseMatrixCSC{Int,Int}}
    I::Vector{String}
    AP::Vector{String}
    LAsMatrix::SparseMatrixCSC{Int,Int}
end

# =========
# Functions
# =========

"""
TransitionSystem(n_S::Integer)
Description:
    An alternative function for defining a new transition system object.
    This implementation creates n_S arbitrary states and nothing else.
"""
function TransitionSystem(n_S::Integer)
    # Constants

    # Algorithm
    tempS::Array{String} = []
    for s in range(1,stop=n_S)
        append!(tempS,[string("s",s)])
    end

    return TransitionSystem(tempS,[], Vector{SparseMatrixCSC{Int,Int}}([]), tempS,[],sparse([],[],[]) )
end

"""
TransitionSystem(n_S::Integer,n_Act::Integer)
Description:
    Creates a transition system with:
    - n_S States
    - n_Act Actions
"""
function System(n_S::Integer,n_Act::Integer)
    # Constants

    # Algorithm

    #s
    tempS::Vector{String} = []
    for s_index in range(1,stop=n_S)
        append!(tempS,[string("s",s_index)])
    end

    # Create n_Act strings for Act
    tempAct::Vector{String} = []
    for act_index in range(1,stop=n_Act)
        append!(tempAct,[string("a_",act_index)])
    end

    #Create empty transition FAsMatrices
    tempTransitionsAsMatrices = Vector{SparseMatrixCSC{Int,Int}}([])
    for s_index in range(1,stop=n_S)
        push!(tempFAsMatrices,sparse([],[],[],n_Act,n_S))
    end

    # Create empty output Matrix
    
    return TransitionSystem(tempS,tempAct, tempTransitionsAsMatrices, tempS,[],sparse([],[],[]) )

end



"""
find_state_index_of(name_in::String, ts_in::TransitionSystem)
Description:
    Retrieves the index of the state that has name name_in.
"""
function find_state_index_of(name_in::String, ts_in::TransitionSystem)
    # Constants

    # algorithm
    for s_index = range(1, stop=length(ts_in.S) )
        if ts_in.S[s_index] == name_in
            return s_index
        end
    end

    # if the name was not found, then return -1.
    return -1

end

"""
find_action_index_of(name_in::String, system_in::TransitionSystem)
Description:
    Retrieves the index of the ACTION that has name name_in.
"""
function find_action_index_of(name_in::String, system_in::TransitionSystem)
    # Constants

    # algorithm
    for act_index = range(1, stop=length(system_in.Act) )
        if system_in.Act[act_index] == name_in
            return act_index
        end
    end

    # if the name was not found, then return -1.
    return -1

end

"""
find_proposition_index_of(name_in::String, system_in::TransitionSystem)
Description:
    Retrieves the index of the AP that has name name_in.
"""
function find_proposition_index_of(name_in::String, system_in::TransitionSystem)
    # Constants

    # algorithm
    for ap_index = range(1, stop=length(system_in.AP) )
        if system_in.AP[ap_index] == name_in
            return ap_index
        end
    end

    # if the name was not found, then return -1.
    return -1

end

"""
Post(s::String, act::String, ts_in::TransitionSystem)
Description:
    Attempts to find the set of states that the system will transition to
    from the current state s with action act.
"""
function Post(s::String, act::String, ts_in::TransitionSystem)
    # Constants
    s_index = find_state_index_of(s,ts_in)
    act_index = find_action_index_of(act,ts_in)

    # Algorithm
    nextStateIndices = Post(s_index,act_index,ts_in) # See below for implementation of F for integers.

    nextStatesAsStrings = Array{String}([])
    for nsi_index = 1:length(nextStateIndices)
        push!(nextStatesAsStrings,ts_in.S[nextStateIndices[nsi_index]])
    end

    return nextStatesAsStrings

end

"""
Post(s_index::Integer, act_index::Integer, ts_in::TransitionSystem)
Description:
    Attempts to find the set of states that the system will transition to
    from the current state system_in.X[x_index] with input system_in.U[u_index].
"""
function Post(s_index::Integer, act_index::Integer, ts_in::TransitionSystem)::Vector{Integer}
    # Constants

    # Algorithm
    T_s = ts_in.Transitions[s_index]
    if T_s == []
        throw(DomainError("No transitions are defined for the system from the state "+string(s_index)+"."))
    end

    tempI, tempJ, tempV = findnz(T_s)
    matching_indices = findall( tempI .== act_index )

    return tempJ[ matching_indices ]

end

"""
Post(s_index::Integer, ts_in::TransitionSystem)
Description:
    Attempts to find the set of states that the system can transition to
    from the current state system_in.X[x_index] with ANY input.
"""
function Post(s_index::Integer, ts_in::TransitionSystem)
    # Constants

    # Algorithm
    temp_post = Vector{Integer}([])
    for act_index in range(1,stop=length(ts_in.Act))
        temp_post = union!(temp_post,Post(s_index,act_index,ts_in))
    end

    return temp_post

end

"""
Post(s::String, ts_in::TransitionSystem)
Description:
    Attempts to find the set of states that the system can transition to
    from the current state s with ANY input.
"""
function Post(s::String, ts_in::TransitionSystem)
    # Constants
    s_index = find_state_index_of(s,ts_in)

    # Algorithm
    temp_post_as_indices = Post(s_index,ts_in)

    # Convert temp_post_as_indices to a Vector of strings
    temp_post = Vector{String}([])
    for temp_s_index in temp_post_as_indices
        temp_post = push!(temp_post,ts_in.S[temp_s_index])
    end

    return temp_post

end

"""
Post(s_array::Vector{String}, act::String, ts_in::TransitionSystem)
Description:
    Attempts to find the set of states that the system will transition to
    from the current state (which is somewhere in s_array) with input act.
"""
function Post(s_array::Vector{String}, act::String, ts_in::TransitionSystem)
    # Constants
    act_index = find_action_index_of(act,ts_in)

    # Algorithm
    nextStateIndices = Vector{Integer}([])
    for s in s_array
        s_index = find_state_index_of(s,ts_in)
        push!(nextStateIndices,Post(s_index,act_index,ts_in)...)
    end

    nextStatesAsStrings = Array{String}([])
    for nsi_index = 1:length(nextStateIndices)
        push!(nextStatesAsStrings,ts_in.S[nextStateIndices[nsi_index]])
    end

    return nextStatesAsStrings

end

"""
Post(s_indices::Vector{Integer}, act_index::Integer, ts_in::TransitionSystem)
Description:
    Attempts to find the set of states that the system will transition to
    from the current state system_in.S[s_index] with input system_in.Act[act_index].
"""
function Post(s_indices::Vector{Integer}, act_index::Integer, ts_in::TransitionSystem)
    # Constants

    # Algorithm
    ancestorStates = Vector{String}([])

    for s_index in s_indices
        push!(ancestorStates,Post(s_index,act_index,system_in)...)
    end

    return ancestorStates

end

"""
add_transition!(ts_in::TransitionSystem,transition_in::Tuple{Int,Int,Int})
Description:
    Adds a transition to the transition system ts_in according to the tuple of indices tuple_in.
        tuple_in = (s_in,act_in,s_next_in)
"""
function add_transition!(ts_in::TransitionSystem,transition_in::Tuple{Int,Int,Int})
    # Constants
    s_in = transition_in[1]
    act_in = transition_in[2]
    s_next_in = transition_in[3]

    # Checking inputs
    check_s(s_in,ts_in)
    check_act(act_in,ts_in)
    check_s(s_next_in,ts_in)
    
    # Algorithm
    ts_in.Transitions[s_in][act_in,s_next_in] = 1
    
end

"""
add_transition!(ts_in::TransitionSystem,transition_in::Tuple{String,String,String})
Description:
    Adds a transition to the transition system ts_in according to the tuple of NAMES tuple_in.
        tuple_in = (s_in,act_in,s_next_in)
"""
function add_transition!(ts_in::TransitionSystem,transition_in::Tuple{String,String,String})
    # Constants
    s_in = transition_in[1]
    act_in = transition_in[2]
    s_next_in = transition_in[3]

    # Checking inputs

    # println(transition_in)
    
    s_index = find_state_index_of(s_in,ts_in)
    act_index = find_action_index_of(act_in,ts_in)
    s_next_index = find_state_index_of(s_next_in,ts_in)

    # Algorithm
    add_transition!(ts_in,( s_index , act_index , s_next_index ))
    
end

"""
check_s(s_in::Integer,ts_in::TransitionSystem)
Description:
    Checks to make sure that a possible state index is actually in the bounds of the set of states S.
"""
function check_s(s_in::Integer,ts_in::TransitionSystem)
    # Constants
    n_S = length(ts_in.S)

    # Algorithm
    if (1 > s_in) || (n_S < s_in)
        throw(DomainError("The input transition references a state " * string(s_in) * " which is not in the state space!"))
    end

    return
end

"""
check_s(s_in::String,ts_in::TransitionSystem)
Description:
    Checks to make sure that a possible state NAME is actually in the set of states S
"""
function check_s(s_in::String,ts_in::TransitionSystem)
    # Constants

    # Algorithm
    if !(s_in in ts_in.S)
        throw(DomainError("The input transition references a state " * string(s_in) * " which is not in the state space!"))
    end

    return
end

"""
check_act(act_index_in::Integer,ts_in::TransitionSystem)
Description:
    Checks to make sure that a possible action INDEX is actually in the bounds of The
    set of actions Act.
"""
function check_act(act_index_in::Integer,ts_in::TransitionSystem)
    # Constants
    n_Act = length(ts_in.Act)

    # Algorithm
    if (1 > act_index_in) || (n_Act < act_index_in)
        throw(DomainError("The input transition references a state " * string(act_index_in) * " which is not in the input space!"))
    end

    return
end

"""
check_act(act_in::String,ts_in::TransitionSystem)
Description:
    Checks to make sure that a possible action name is actually in the bounds of the
    set of all actions Act.
"""
function check_act(act_in::String,ts_in::TransitionSystem)
    # Constants

    # Algorithm
    if !(act_in in ts_in.Act)
        throw(DomainError("The input transition references a state " * string(act_in) * " which is not in the input space!"))
    end

    return
end

"""
get_vending_machine_system()
Description:
    Returns the beverage vending machine example.
"""
function get_vending_machine_system1()
    # Constants
    state_names = ["pay","select","get_beer","get_soda"]
    input_names = ["N/A"]
    output_names = ["pay","select","getting_drink"]

    # Algorithm
    system_out = System(length(state_names),length(input_names),length(output_names))
    
    # Add state names
    for state_index in range(1,stop=length(state_names))
        system_out.X[state_index] = state_names[state_index]
    end

    # Add Input Names
    for input_index in range(1,stop=length(input_names))
        system_out.U[input_index] = input_names[input_index]
    end

    # Add Output Names
    for output_index in range(1,stop=length(output_names))
        system_out.Y[output_index] = output_names[output_index]
    end

    # Create transitions
    add_transition!(system_out,("pay","N/A","select"))
    add_transition!(system_out,("select","N/A","get_beer"))
    add_transition!(system_out,("select","N/A","get_soda"))
    add_transition!(system_out,("get_beer","N/A","pay"))
    add_transition!(system_out,("get_soda","N/A","pay"))

    # Create Outputs
    system_out.HAsMatrix[1,1] = 1
    system_out.HAsMatrix[2,2] = 1
    system_out.HAsMatrix[3,3] = 1
    system_out.HAsMatrix[4,3] = 1

    return system_out
end

"""
get_figure2_system(num_b::Integer)
Description:
    Returns the discrete state system example from Figure 2.
"""
function get_figure2_system(num_b::Integer)
    # Constants
    input_names = ["N/A"]
    output_names = ["A","B"]

    num_a = 2

    # Input Processing
    if num_b < 1
        throw(DomainError("The number of 'b' states must be a positive integer, not " * string(num_b) * "!"))
    end

    # Create System

    system_out = System(num_a+num_b,length(input_names),length(output_names))
    
    # Add state names
    system_out.X[1] = "a1"
    system_out.X[2] = "a2"
    for state_index in range(1,stop=num_b)
        system_out.X[state_index+2] = "b" * string(state_index)
    end

    # Add Input Names
    for input_index in range(1,stop=length(input_names))
        system_out.U[input_index] = input_names[input_index]
    end

    # Add Output Names
    for output_index in range(1,stop=length(output_names))
        system_out.Y[output_index] = output_names[output_index]
    end

    # Add Initial States
    push!(system_out.X0,"a1","a2")

    # Create transitions
    add_transition!(system_out,("a1","N/A","b1"))
    for b_index in range(2,stop=num_b)
        add_transition!(system_out,( "b" * string(b_index-1), "N/A" , "b" * string(b_index)  ))
        add_transition!(system_out,( "b" * string(b_index)  , "N/A" , "b" * string(b_index-1)  ))
        add_transition!(system_out,( "b" * string(b_index)  , "N/A" , "a2"  ))
    end

    add_transition!(system_out,("b1","N/A","a2"))
    add_transition!(system_out,("a2","N/A","a2"))

    # Create Outputs
    for x_index in range(1,stop=length(system_out.X))
        x = system_out.X[x_index]

        if (x == "a1") || (x == "a2")
            # Label with an A
            system_out.HAsMatrix[x_index,1] = 1
        else
            # Label with a B
            system_out.HAsMatrix[x_index,2] = 1
        end

    end

    return system_out
end

"""
get_figure3_system(num_patterns::Integer)
Description:
    Returns the discrete state system example from Figure 3 of the paper.
"""
function get_figure3_system(num_patterns::Integer)
    # Constants
    input_names = ["N/A"]
    output_names = ["A","B","C","D","E","F","G"]

    # Input Processing
    if num_patterns < 1
        throw(DomainError("The number of 'pattern repititions' states must be a positive integer, not " * string(num_patterns) * "!"))
    end

    # Create System
    n_a = 1
    n_b = 6 * num_patterns
    n_c = 6 * num_patterns
    n_d = ( 1 + 2 + 2 ) * num_patterns
    n_e = ( 1 + 2 + 2 ) * num_patterns
    n_f = 6 * num_patterns
    n_g = 6 * num_patterns

    n_list = [ n_a , n_b , n_c , n_d , n_e , n_f , n_g ]

    n_X = sum(n_list)

    system_out = System(n_X,length(input_names),length(output_names))
    
    # Add state names
    system_out.X[1] = "a1"
    temp_prefixes = ["a","b","c"]
    for prefix_index in range(1,stop=length(temp_prefixes))
        for state_index in range(1,stop=n_list[prefix_index])
            system_out.X[state_index+sum(n_list[1:prefix_index-1])] = temp_prefixes[prefix_index] * string(state_index)
        end
    end

    # insert d names
    for state_index in range(1,stop=n_d)
        if mod(state_index,5) == 1
            system_out.X[state_index+sum(n_list[1:3])] = "d" * string(state_index + div(state_index,5,RoundDown))
        elseif (mod(state_index,5) == 2)
            system_out.X[state_index+sum(n_list[1:3])] = "d" * string(state_index+1 + div(state_index,5,RoundDown)) * "^l"
        elseif mod(state_index,5) == 3
            system_out.X[state_index+sum(n_list[1:3])] = "d" * string(state_index + div(state_index,5,RoundDown)) * "^r"
        elseif mod(state_index,5) == 4
            system_out.X[state_index+sum(n_list[1:3])] = "d" * string(state_index+1 + div(state_index,5,RoundDown)) * "^l"
        elseif mod(state_index,5) == 0
            system_out.X[state_index+sum(n_list[1:3])] = "d" * string(state_index + div(state_index,5,RoundDown)-1) * "^r"
        end
    end

    # insert e names
    for state_index in range(1,stop=n_e)
        if mod(state_index,5) == 1
            system_out.X[state_index+sum(n_list[1:4])] = "e" * string(state_index+1 + div(state_index,5,RoundDown) ) * "^l"
        elseif (mod(state_index,5) == 2)
            system_out.X[state_index+sum(n_list[1:4])] = "e" * string(state_index + div(state_index,5,RoundDown) ) * "^r"
        elseif mod(state_index,5) == 3
            system_out.X[state_index+sum(n_list[1:4])] = "e" * string(state_index+1 + div(state_index,5,RoundDown) )
        elseif mod(state_index,5) == 4
            system_out.X[state_index+sum(n_list[1:4])] = "e" * string(state_index+2 + div(state_index,5,RoundDown) ) * "^l"
        elseif mod(state_index,5) == 0
            system_out.X[state_index+sum(n_list[1:4])] = "e" * string(state_index+1 + div(state_index,5,RoundDown)-1 ) * "^r"
        end
    end

    # insert f and g names
    temp_prefixes2 = ["f","g"]
    for prefix_index in range(1,stop=length(temp_prefixes2))
        for state_index in range(1,stop=n_list[5+prefix_index])
            system_out.X[state_index+sum(n_list[1:5+prefix_index-1])] = temp_prefixes2[prefix_index] * string(state_index)
        end
    end

    # for state_index in range(1,stop=length(system_out.X))
    #     println(system_out.X[state_index])
    # end

    # Add Input Names
    for input_index in range(1,stop=length(input_names))
        system_out.U[input_index] = input_names[input_index]
    end

    # Add Output Names
    for output_index in range(1,stop=length(output_names))
        system_out.Y[output_index] = output_names[output_index]
    end

    # Add Initial States
    push!(system_out.X0,"a1")

    # Create transitions
    add_transition!(system_out,("a1","N/A","b1"))
    # Add b transitions
    for b_index in range(1,stop=n_b-1)
        add_transition!(system_out,( "b" * string(b_index+1), "N/A" , "b" * string(b_index)  ))
        add_transition!(system_out,( "b" * string(b_index)  , "N/A" , "b" * string(b_index+1)  ))
        add_transition!(system_out,( "b" * string(b_index)  , "N/A" , "c" * string(b_index)  ))
    end
    # Add c transitions
    for c_index in range(1,stop=n_c)
        if mod(c_index,6) == 1
            # Send to the a state marked d * string(c_index)
            add_transition!(system_out,( "c" * string(c_index)  , "N/A" , "d" * string(c_index)  ))
        elseif mod(c_index,6) == 2
            # Send to the left and right e states
            add_transition!(system_out,( "c" * string(c_index)  , "N/A" , "e" * string(c_index) * "^l" ))
            add_transition!(system_out,( "c" * string(c_index)  , "N/A" , "e" * string(c_index) * "^r" ))
        elseif mod(c_index,6) == 3
            # Send to the left and right d states
            add_transition!(system_out,( "c" * string(c_index)  , "N/A" , "d" * string(c_index) * "^l" ))
            add_transition!(system_out,( "c" * string(c_index)  , "N/A" , "d" * string(c_index) * "^r" ))
        elseif mod(c_index,6) == 4
            # Send to the a state marked e * string(c_index)
            add_transition!(system_out,( "c" * string(c_index)  , "N/A" , "e" * string(c_index)  ))
        elseif mod(c_index,6) == 5
            # Send to the left and right d states
            add_transition!(system_out,( "c" * string(c_index)  , "N/A" , "d" * string(c_index) * "^l" ))
            add_transition!(system_out,( "c" * string(c_index)  , "N/A" , "d" * string(c_index) * "^r" ))
        elseif mod(c_index,6) == 0
            # Send to the left and right e states
            add_transition!(system_out,( "c" * string(c_index)  , "N/A" , "e" * string(c_index) * "^l" ))
            add_transition!(system_out,( "c" * string(c_index)  , "N/A" , "e" * string(c_index) * "^r" ))
        end
    end

    # add d transitions
    for d_index in range(1,stop=n_d)
        if mod(d_index,5) == 1
            # Send to both f and g states with d_index
            add_transition!(system_out,( "d" * string(d_index + div(d_index,5,RoundDown) )  , "N/A" , "f" * string(d_index + div(d_index,5,RoundDown))  ))
            add_transition!(system_out,( "d" * string(d_index + div(d_index,5,RoundDown) )  , "N/A" , "g" * string(d_index + div(d_index,5,RoundDown))  ))
        elseif mod(d_index,5) == 2
            # This is the left state. Send to the f state
            add_transition!(system_out,( "d" * string(d_index+1 + div(d_index,5,RoundDown) ) * "^l"  , "N/A" , "f" * string(d_index+1 + div(d_index,5,RoundDown) ) ))
        elseif mod(d_index,5) == 3
            # This is the right state. Send to the g state with d_index
            add_transition!(system_out,( "d" * string(d_index + div(d_index,5,RoundDown)) * "^r"  , "N/A" , "g" * string(d_index + div(d_index,5,RoundDown)) ))
        elseif mod(d_index,5) == 4
            # This is the left state. Send to the f state
            add_transition!(system_out,( "d" * string(d_index+1 + div(d_index,5,RoundDown) ) * "^l"  , "N/A" , "f" * string(d_index+1 + div(d_index,5,RoundDown) )) )
        elseif mod(d_index,5) == 0
            # This is the right state. Send to the g state with d_index
            add_transition!(system_out,( "d" * string(d_index + div(d_index,5,RoundDown) - 1 ) * "^r"  , "N/A" , "g" * string(d_index + div(d_index,5,RoundDown) -1 ) ))
        end
    end

    # add e transitions
    for e_index in range(1,stop=n_e)
        if mod(e_index,5) == 1
            # This is the left state. Route to f
            add_transition!(system_out,( "e" * string(e_index+1 + div(e_index,5,RoundDown) ) * "^l"  , "N/A" , "f" * string(e_index+1 + div(e_index,5,RoundDown) ) ))
        elseif mod(e_index,5) == 2
            # This is the right state. Send to the g state with e_index
            add_transition!(system_out,( "e" * string(e_index + div(e_index,5,RoundDown) ) * "^r"  , "N/A" , "g" * string(e_index + div(e_index,5,RoundDown) ) ))
        elseif mod(e_index,5) == 3
            # Send to both f and g states with e_index
            add_transition!(system_out,( "e" * string(e_index+1 + div(e_index,5,RoundDown) )  , "N/A" , "f" * string(e_index+1 + div(e_index,5,RoundDown) )  ))
            add_transition!(system_out,( "e" * string(e_index+1 + div(e_index,5,RoundDown) )  , "N/A" , "g" * string(e_index+1 + div(e_index,5,RoundDown) )  ))
        elseif mod(e_index,5) == 4
            # This is the left state. Send to the f state
            add_transition!(system_out,( "e" * string(e_index+2 + div(e_index,5,RoundDown)) * "^l"  , "N/A" , "f" * string(e_index+2 + div(e_index,5,RoundDown) ) ))
        elseif mod(e_index,5) == 0
            # This is the right state. Send to the g state with e_index
            add_transition!(system_out,( "e" * string(e_index+1 + div(e_index,5,RoundDown) - 1) * "^r"  , "N/A" , "g" * string(e_index+1 + div(e_index,5,RoundDown) - 1 ) ))
        end

    end

    # add f and g transitions
    for state_index in range(1,stop=n_f)
        add_transition!(system_out,( "f" * string(state_index) , "N/A" , "f" * string(state_index) ))
        add_transition!(system_out,( "g" * string(state_index) , "N/A" , "g" * string(state_index) ))
    end

    # add_transition!(system_out,("b1","N/A","a2"))
    # add_transition!(system_out,("a2","N/A","a2"))

    # Create Outputs
    temp_prefixes3 = ["a","b","c","d","e","f","g"]
    for x_index in range(1,stop=length(system_out.X))
        x = system_out.X[x_index]

        # Search for matches in temp_prefixes3
        for prefix_index in range(1,stop=length(temp_prefixes3))
            if contains( x , temp_prefixes3[prefix_index] )
                system_out.HAsMatrix[x_index,prefix_index] = 1
            end

        end

    end

    return system_out
end

"""
to_graph(ts_in::TransitionSystem)
Description:
    Converts the given transition system to a graph object from Graphs library.
"""
function to_graph(ts_in::TransitionSystem)
    # Constants
    n_S = length(ts_in.S)

    # Algorithm

    # Create a graph
    ts_as_graph = DiGraph(n_S)

    # Iterate through every state's transition Matrix
    for s_index in range(1,stop=n_S)
        # Extract all nonzero transitions
        post_s = Post(s_index,ts_in)

        # Add edges for each element in post_s
        for s_next_ind in post_s
            add_edge!(ts_as_graph,s_index,s_next_ind)
        end
    end

    return ts_as_graph

end

"""
plot(ts_in::TransitionSystem)
Description:

"""
function plot(ts_in::TransitionSystem)
    # Constants

    # Algorithm

    # Convert to Graph
    ts_as_graph = to_graph(ts_in)

    # Get all edges and label them
    num_nodes = length(ts_in.S)
    edge_labels = Vector{String}([])
    for src_index in range(1,stop=num_nodes)
        for dest_index in ts_as_graph.fadjlist[src_index]
            # Find the actions that lead to this transition
            T_s = ts_in.Transitions[src_index]
            tempActions, tempJ, tempV = findnz(T_s)
            matching_action_indices = tempActions[findall( tempJ .== dest_index )] # Get all action indices that lead to the transition from src_index to dest_index

            # Create label by iterating through each matching action
            temp_label = string("")
            for action_index in matching_action_indices
                temp_label = string(temp_label,ts_in.Act[action_index])

                if action_index != last(matching_action_indices)
                    temp_label = string(temp_label,string(","))
                end
            end
            
            push!(edge_labels,temp_label)
        end
    end

    return gplot(ts_as_graph,
                    nodelabel=ts_in.S,
                    edgelabel=edge_labels)

end