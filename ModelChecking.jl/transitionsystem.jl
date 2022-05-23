"""
transitionSystem.jl
"""

struct TransitionSystemState
    Name::String
    # System::TransitionSystem
end

struct TransitionSystem
    S::AbstractVector{AbstractString}
    Act::AbstractVector{AbstractString}
    Transitions::AbstractVector{AbstractMatrix{Int}}
    I::AbstractVector{String}
    AP::AbstractVector{String}
    L::AbstractVector{AbstractVector{AbstractString}}
    Y::AbstractVector{AbstractString}
end

# =========
# Functions
# =========

function GetTransitionSystem1( nS::Int, nAct::Int,transitionMatrix::Vector{Matrix{Int}},IIn::Vector{Int},nAP::Int,LIn::Vector{Vector{Int}})::TransitionSystem
    # Constants

    # Create the state Vector
    if nS < 1
        throw("The number of states must be greater than 0.")
    end

    tempS = Vector{String}([])
    for tempInt in range(1,nS,step=1)
        push!(tempS,string("s",tempInt) )
    end

    # Create the Act Vector
    tempAct = Vector{String}([])
    for tempInt in range(1,nAct,step=1)
        push!(tempAct,string("a",tempInt))
    end

    # Create the Transition Map
    # (This is not necessary)
    tempTransitionMatrix = transitionMatrix

    # Create the set of initial states
    tempI = Vector{String}([])
    for tempIndex in IIn
        push!(tempI,string("s",tempIndex))
    end

    # Create the Atomic Proposition Vector
    tempAP = Vector{String}([])
    for tempInt in range(1,nAP,step=1)
        push!(tempAP,string("AP",tempInt))
    end

    # Create the Labelling Function
    tempL = Vector{Vector{String}}([])
    for stateIndex in range(1,length(tempS),step=1)
        labelsFor_s = Vector{String}([])
        for labelIndex in LIn[stateIndex]
            push!(labelsFor_s,tempAP[labelIndex])
        end
        push!(tempL,labelsFor_s)
    end

    return TransitionSystem(
        tempS,
        tempAct,
        tempTransitionMatrix,
        tempI,
        tempAP,
        tempL,
        tempS
    )
    println(tempAP)


end

function Check(tsIn::TransitionSystem)
    
end