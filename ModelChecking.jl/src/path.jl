# path.jl
# Description:
#   A file containing some of the structures needed to represent and manipulate the
#   Trace object of a model checking algorithm.

const FinitePath = Vector{String,1}

struct InfinitePath
    FinitePrefix::Path
    RepeatingSuffix::Path
end

function check( pi::FinitePath, ts_in::TransitionSystem)
    """
    check
    Description:
        Checks to see if the transitions provided in 
    """
    
    # Checking all of the transitions are valid
    if length(pi) > 1
        for pi_index in range(1,length(pi)-1)
            # Get the current state and then the next one
            s_i = pi[pi_index]
            s_ip1 = pi[pi_index+1]
            if !(s_ip1 in Post(s_i))
                throw ErrorException("The transition from state " + ts_in.S[s_i] + " does not exist")

        end
    end

end