%test2_mat.m
%Description:
%	Tests the verification step of the Adaptive Control Barrier Function Paper.

%% Constants

accSystem = AdaptiveCruiseControlSystem();

alpha = 3*12;

syms v D theta1 theta2 theta3
h_a = alpha^2 - (D - 1.8*v - alpha)^2;

h_a_func = @(x,theta) alpha^2 - (x(1) - 1.8 * x(2) - alpha)^2;

n_p = 3; %Number of unknown parameters.
Gamma = eye(n_p);

%% Algorithm

x1 = [ 10 ; 20 ];
x2 = [ 50 ; 20 ];
lambda_cbf(accSystem , h_a_func , eye(3) ,  x1 , [ accSystem.f0 ; accSystem.f1 ; accSystem.f2 ] )

disp('Computing gradient with my sparkly new function: ')
[dh_a_dx, dh_a_dtheta] = gradient_wrt_x_at( h_a_func , x1 , [ accSystem.f0 ; accSystem.f1 ; accSystem.f2 ] )

[ k_x_theta ] = acbf_controller1( accSystem , h_a_func , Gamma , x2 , [ accSystem.f0 + 0.1 ; accSystem.f1+0.001 ; accSystem.f2+0.001 ] )

function [ k_x_theta ] = acbf_controller1( system_in , h_a , Gamma , x , theta_hat )
	%Description:
	%
	%Usage:
	%
	%Inputs:
	%	system
	%	h_a - The adaptive CBF that you've found. (ideally you should verify that this is a true aCBF)
	%	Gamma - A symmetric matrix p x p of the same dimension as your parameter.
	%	x - The current state.
	%	theta_hat - The current estimate of the unknown paramter.

	%% Compute the gradient of the barrier at the current time instant.
	%Description:
	%	Computes the output of the optimization problem formed by the adaptive Control Barrier Function optimization

	% Constants
	[dh_a_dx, dh_a_dtheta] = gradient_wrt_x_at( h_a , x , theta_hat );

	% Create the Simplifying Matrices
	f_tilde_cbf = system_in.f(x) + system_in.F(x) * lambda_cbf( system_in , h_a , Gamma, x , theta_hat )

	% Construct Optimization
	u = sdpvar(1,1);

	barrier_constraint = [ dh_a_dx' * ( f_tilde_cbf + system_in.g(x) * u ) >= 0 ];

	objective = 0.5 * norm(u,2);

	ops = sdpsettings('verbose',1);
    opt0 = optimize(barrier_constraint,objective,ops);

    k_x_theta = value(u);
	
end

function [dh_a_dx, dh_a_dtheta] = gradient_wrt_x_at( h_a , x , theta )
	%Description:
	%	Gradient of the scalar function h_a.
	%Assumptions:
	%	- Assumes that the function receives as input the state x and the parameters theta
	%Inputs:
	%	x - State to compute gradient w.r.t.
	%	theta - parameter vector theta to compute gradient w.r.t.
	%
	%Usage:
	%	[dh_a_dx, dh_a_dtheta] = gradient_wrt_x_at( h_a , x , theta_hat );


	% Constants
	n_x = length(x);
	n_p = length(theta); %Dimension of the parameter vector theta

	%Compute the gradient of h_a with respect to theta
	theta_vector_string = '[';
	for theta_index = 1:n_p
		theta_vector_string = [ theta_vector_string 'theta_' num2str(theta_index) ];

		if theta_index ~= n_p
			theta_vector_string = [ theta_vector_string  ',' ];
		end
	end
	theta_vector_string = [ theta_vector_string  ']' ];
	theta_symbolic = str2sym(theta_vector_string);

	x_vector_string = '[';
	for x_index = 1:n_p
		x_vector_string = [ x_vector_string 'x_' num2str(x_index) ];

		if theta_index ~= n_x
			x_vector_string = [ x_vector_string  ',' ];
		end
	end
	x_vector_string = [ x_vector_string  ']' ];
	x_symbolic = str2sym(x_vector_string)

	% Compute h_a as a symbolic scalar and then use a toolbox to compute gradient
	h_a_symbolic = h_a(x_symbolic,theta_symbolic)
	gradient_h_a = gradient(h_a_symbolic);

	if length(gradient_h_a) ~= n_x + n_p
		gradient_h_a(n_x+n_p) = 0;
	end

	% Evaluate gradient at x and theta
	eval_gradient_h_a = gradient_h_a
	% Over x
	for x_index = 1:n_x
		eval_gradient_h_a = subs(eval_gradient_h_a,['x_' num2str(x_index)],x(x_index))
	end	

	for theta_index = 1:n_p
		eval_gradient_h_a = subs(eval_gradient_h_a,['theta_' num2str(theta_index)],theta(theta_index))
	end

	dh_a_dx = double(eval_gradient_h_a(1:n_x));
	dh_a_dtheta = double(eval_gradient_h_a(n_x+1:end));

end

function [ lambda_out ] = lambda_cbf( system , h_a , Gamma , x , theta_hat )
	%Description:
	%
	%Usage:
	%

	% Constants
	n_x = system.n_x;
	n_u = system.n_u;
	n_p = system.n_theta;

	if size(Gamma,1) ~= size(Gamma,2)
		error('Gamma is not square. It should be!')
	end

	if n_p ~= size(Gamma,1)
		error('Gamma''s dimensions do not match that of the system''s parameter size.')
	end

	% Algorithm

	% Compute gradient w.r.t. theta_hat
	[~, dh_a_dtheta] = gradient_wrt_x_at( h_a , x , theta_hat );

	% Perform Multiplication 

	lambda_out = theta_hat - Gamma * (dh_a_dtheta);

end
