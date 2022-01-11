"""
switchedaffinesystem.jl
Description:
    This is a simple representation of a switched affine system whose switching is
    governed by a Language object.
"""

using Polyhedra

struct SwitchedAffineSystem
    Modes::Vector{AffineSystem}
    L::Language
    X0::Polyhedron{Float64}
    U::Polyhedron{Float64}
end

# Functions

function check_modes(sas_in::SwitchedAffineSystem)

    # Check to see if Modes or L are empty
    if length(sas_in.Modes) < 1
        throw(BoundsError("There must be a nonzero number of modes in the SwitchedAffineSystem. Found " * string(length(sas_in.Modes))))
    end

    return

end