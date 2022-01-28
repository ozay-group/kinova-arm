"""
    linear_system.jl
    Description:
        Defines the set of functions that help us work with linear systems.
        Represents a discrete-time linear dynamical system as follows:
         x(t+1) = A x(t) + B u(t) + w(t) + K
         y(t)   = C x(t) + v(t)
         z(t)   = D x(t) + d
        where 
            w(t) in { w in R^n | W.A * w <= W.b },
            v(t) in { v in R^p | V.A * v <= V.b }, and 
            u(t) in { u in R^m | U.A * u <= U.b }.
"""

using LinearAlgebra
using JuMP
using Gurobi
using Polyhedra

# Todo: Make the struct require SimplePolyhedron or Interval arguments for W, V, and U.
#       Not sure how to make an input required to have a class within a set of types. :/

struct AffineSystem
    A::Matrix{Float64}
    B::Matrix{Float64}
    C::Matrix{Float64}
    K::Matrix{Float64}
    W::Polyhedron{Float64} # The bounded set that process noise can come from
    V::Polyhedron{Float64} # The bounded set that measurement noise can come from
    U::Polyhedron{Float64} # The Polyhedron that the inputs are allowed to come from
end

"""
    check
    Description:   
        Checks that the system described by the tuple AffineSystem is consistent.
"""
function check( system_in::AffineSystem )
    # Declare Variables
    n = 1
    m = 1
    p = 1
    
    #############
    # Algorithm #
    #############

    # Check System Matrices
    sizeA = size(system_in.A)
    if length(sizeA) ≠ 0 #If A is not a scalar, then apply the following checks.
        n_x1 , n_x2 = sizeA
        if n_x1 ≠ n_x2
            #This indicates that system_in.A is not square
            return false
        end
        n = n_x1 # Assign dimension.
    end

    sizeC = size(system_in.C)
    if length(sizeC) ≠ 0 #If C is not a scalar, then apply the following checks.
        n_y, n_x3 = sizeC
        if n_x1 ≠ n_x3
            #This indicates that system_in.A is of different dimension than Checks
            return false
        end
        p = n_y # Assign dimension of the output.
    end

    # Check the Type of Disturbance Sets
    # Not needed now that struct restricts types

    #All Checks Passed
    return true
end

"""
    x_dim
    Description:
        Computes the dimension of the system.
"""
function x_dim( system_in::AffineSystem )::Integer

    if !check( system_in )
        throw(ArgumentError("The input AffineSystem is not valid. Please check its dimensions."))
    end

    return size(system_in.A,1)

end

"""
    y_dim
    Description:
        Computes the dimension of the system's outputs.
"""
function y_dim( system_in::AffineSystem )::Integer

    if !check( system_in )
        throw(ArgumentError("The input AffineSystem is not valid. Please check its dimensions."))
    end

    return size(system_in.C,1)

end

"""
    u_dim
    Description:
        Computes the dimension of the system's outputs.
"""
function u_dim( system_in::AffineSystem )

    if !check( system_in )
        throw(ArgumentError("The input AffineSystem is not valid. Please check its dimensions."))
    end

    return size(system_in.B,2)
end

"""
    packet_loss_observability_mat
    Description:
        Computes the Observability Matrix with respect to the packet loss signal sigma
        as defined in 'On Observability in Networked Control Systems with Packet Losses'
        [https://doi-org.proxy.lib.umich.edu/10.1109/ALLERTON.2015.7447010].
    Usage:
        O_t = packet_loss_observability_mat( sys0 , [1,0,0,1] )
"""
function packet_loss_observability_mat( system_in::AffineSystem , sigma_in )
    
    # Input Checking

    if length(sigma_in) == 0
        throw(ArgumentError("The input sigma_in should be a binary vector with length at least 1."))
    end

    # Algorithm

    O_t = sigma_in[1]*system_in.C
    for t = 1:(length(sigma_in)-1)
        O_t = [ O_t ; sigma_in[t+1]*system_in.C * system_in.A^t ]
    end

    return O_t
end

"""
    define_simple_eta_HPolyhtope
    Description:
        Defines a polytope containing all possible choices of eta. In math, eta is:
              [  w  ]
        eta = [  v  ]
              [ x0  ]
        Or more completely:
              [  w(0)  ]
              [  w(1)  ]
              [  w(2)  ]
              [  ....  ]
              [ w(T-1) ]
        eta = [  v(0)  ]
              [  v(1)  ]
              [  ....  ]
              [ v(T-1) ]
              [   x0   ]
        where T is T_Horizon_in.

    Usage:
        H_eta , h_eta = define_simple_eta_HPolyhtope( system_in , P_x0 , T )
"""
function define_simple_eta_HPolytope( system_in::AffineSystem , P_x0::Polyhedron{Float64} , T_Horizon_in )
    #Constants
    n_x = x_dim(system_in)
    n_y = y_dim(system_in)

    n_w = n_x
    n_v = n_y

    T = T_Horizon_in

    #Create the Polytopes defined by system_in.eta_w and system_in.eta_v
    hrep_W = MixedMatHRep(system_in.W)
    H_w = hrep_W.A
    h_w = hrep_W.b
    dim_H_w = length(h_w)

    hrep_V = MixedMatHRep(system_in.V)
    H_v = hrep_V.A
    h_v = hrep_V.b
    dim_H_v = length(h_v)

    hrep_X0 = MixedMatHRep(P_x0)
    H_x0 = hrep_X0.A
    h_x0 = hrep_X0.b
    dim_H_x0 = length(h_x0)

    #Algorithm
    H_eta = [kron(Matrix(1I,T,T), H_w) zeros(T*dim_H_w,T*n_y + n_x) ;
             zeros(T*dim_H_v, T*n_x) kron(Matrix(1I,T,T), H_v) zeros(T*dim_H_v,n_x);
             zeros(dim_H_x0,T*(n_x+n_y)) H_x0]

    h_eta = [kron(ones(T,1),h_w); kron(ones(T,1),h_v); h_x0];

    return H_eta, h_eta

end

"""
    find_est_error_bound_from_time_0_to_T
    Description:
        Finds the bound B which bounds all estimation error values between time t=0
        and time T.
"""
function find_est_error_bound_from_time_0_to_T( system_in::AffineSystem , T::Int , eta_x0 )

    #Define Constants
    model = Model(Gurobi.Optimizer) # Create Optimization model
    H_eta, h_eta = define_simple_eta_HPolytope( system_in , eta_x0 , T ) # Create Eta Polyhedron (Polyhedron of all disturbances over time horizon)
    n_Heta = size(H_eta,1)

    n_x = x_dim(system_in)
    n_y = y_dim(system_in)

    S = compute_Sw( system_in.A , T )
    # C_bar = compute_C_M( system_in.C , [T-1] , T )
    J = compute_Sx0( system_in.A , T)

    R_T = Matrix{Float64}([zeros(n_x,T*n_x) I(n_x)])

    #Create the Optimization Variables
    @variable(model,alpha0)
    @variable(model,Lambda[1:2*n_x,1:n_Heta])

    # Create Constraints
    @constraint(model,alpha0.>=0)
    @constraint(model,Lambda.>=zeros(size(Lambda)))

    LHS1 = Lambda * H_eta
    # println(typeof(S))
    # println(typeof(J))
    # println(typeof([ S  zeros(size(S,1),n_y*T) J ]))
    # println(S)
    # println(typeof(R_T))
    # println( R_T*[ S  zeros(size(S,1),n_y*T) J ] )
    RHS1 = [I(n_x); -I(n_x)] * R_T * [ S  zeros(size(S,1),n_y*T) J ]
    @constraint(model, LHS1 .== RHS1 )

    LHS2 = Lambda * h_eta
    RHS2 = alpha0*ones(2*n_x,1)
    @constraint(model, LHS2 .<= RHS2)

    # Create objective
    @objective(model, Min, alpha0)

    #Optimize!
    set_silent(model)
    optimize!(model)

    return termination_status(model) , objective_value(model)

end

"""
    evaluate_schedule_wrt
    Description:
        Evaluates the schedule with respect to (i) a desired linear system, (ii) the schedule
        (iii) the time horizon, and (iv) one of several objectives including:
            1 - Maximum Infinity Norm of Estimation error 
            2 - Sum of the Estimation Error bounds at each time
    Usage:
        obj_val0 , feas_flag0 = evaluate_schedule_wrt( ls0, schedule0 , T0 , 1 , eta_x0 )
        
        
"""
function evaluate_schedule_wrt( system_in::AffineSystem , schedule_in , time_horizon_in , objective_flag::Int , x0_description )

    # Input Processing

    if !check( system_in )
        throw(ArgumentError("The input AffineSystem is not valid. Please check its dimensions."))
    end

    for schedule_val in schedule_in
        if schedule_val > time_horizon_in
            throw(ArgumentError("The schedule given contains times which are outside of the time horizon! (" + str(schedule_val) + ")" ))
        end
    end

    expected_objective_flag_range = (1,2)

    if (objective_flag < expected_objective_flag_range[1]) | (objective_flag > expected_objective_flag_range[2])
        throw(ArgumentError("The input objective_flag is not in the desired range $(expected_objective_flag_range).\n"))
    end 

    # Constants

    eta_x0 = x0_description #Fix this later when eta_description becomes a polytope?
    H_eta, h_eta = define_simple_eta_HPolytope( system_in , eta_x0 , time_horizon_in ) # Create Eta Polyhedron (Polyhedron of all disturbances over time horizon)
    n_Heta = size(H_eta,1)

    n_x = x_dim(system_in)
    n_y = y_dim(system_in)

    #Clear the model?
    empty!(model)

    #Create the Optimization Variables
    @variable(model,Q[1:n_x*time_horizon_in,1:n_y*time_horizon_in])
    @variable(model,alpha0[1:time_horizon_in+1,1])
    @variable(model,alpha1)

    #Constraints
    @constraint(model,alpha0.>=0)
    @constraint(model,alpha1.>=0)

    S = compute_Sw( system_in.A , time_horizon_in )
    C_bar = compute_C_M( system_in.C , schedule_in , time_horizon_in )
    J = compute_Sx0( system_in.A , time_horizon_in)

    P_xi_w = S + S*Q*C_bar*S
    P_xi_v = S * Q 
    P_xi_xi0 = ( I + S*Q*C_bar)*J
    P_xiTilde = 0

    R_T_mT = [ I ; -Matrix(1I, n_x*(time_horizon_in+1), n_x*(time_horizon_in+1)) ]
    
    R_0_T = I( n_x*(time_horizon_in+1) )
    R_T = [ zeros(n_x,n_x*time_horizon_in) I(n_x) ]

    if objective_flag == 1 #Maximum Estimation Error Bound Objective
        #
        @variable(model,Lambda[1:2*n_x*(time_horizon_in+1),1:n_Heta])
        @constraint(model,Lambda.>=zeros(size(Lambda)))

        LHS1 = Lambda * H_eta
        RHS1 = [I(n_x*(time_horizon_in+1)) ; -I(n_x*(time_horizon_in+1))] * R_0_T * [ P_xi_w P_xi_v P_xi_xi0 ]
        @constraint(model, LHS1 .== RHS1 )

        LHS2 = Lambda * h_eta
        RHS2 = alpha1 * ones(2*n_x*(time_horizon_in+1),1)
        @constraint(model, LHS2 .<= RHS2)

    elseif objective_flag == 2 #Sum of estimation error bounds at each time steps

        # Variable Definitions
        @variable(model,Lambda[1:2*n_x*(time_horizon_in+1),1:n_Heta])
        @constraint(model,Lambda.>=zeros(size(Lambda)))

        LHS1 = Lambda * H_eta
        RHS1 = [I(n_x*(time_horizon_in+1)) ; -I(n_x*(time_horizon_in+1))] * R_0_T * [ P_xi_w P_xi_v P_xi_xi0 ]
        @constraint(model, LHS1 .== RHS1 )

        LHS2 = Lambda * h_eta
        RHS2 = kron(alpha0,ones(2*n_x))
        @constraint(model, LHS2 .<= RHS2)

    end

    # Define Objective

    if objective_flag == 1
        @objective(model,Min,alpha1)
    elseif objective_flag == 2
        @objective(model,Min,sum(alpha0))
    end

    # Optimize!
    optimize!(model)

    print("termination_status = $(termination_status(model))\n")

    return objective_value(model) , termination_status(model)

end

"""
    evaluate_schedule_cost
    Description:
        Evaluates the schedule cost with respect to
        (i) an input linear system (system_in)
        (ii) a measurement schedule (m_schedule),
        (iii) a control schedule (c_schedule),
        and tries to find the minimal value of alpha (the worst case bound on the estimation error)
"""