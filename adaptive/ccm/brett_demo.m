%brett_demo.m
%Description:
%	

clear all;
close all;
clc;

%% Constants %%

%% Define Dynamics %%

n = 3; % System Dimension
x_s = sym('x',[n,1]);
p = 3;
% th = sym('theta',[p,1])

x0 = zeros(n,1);

f = @(x) [x(3); x(1)^2- x(2); tanh(x(2)) ];
Delta = @(x) [ [x(1);0;0] , [ 0 , 0 ; 0 , 0 ; 1 , 1 ] ];
B = @(x) [0;0;1];

dx_dt = @(x,theta,u) f(x) - Delta(x) * theta + B(x) * u;

%% Finding the Parameter Dependent Dual Contraction Metric %%

lambda0 = 0.1;

B0 = B(x0);
B0_perp = null(B0');

df_dx = jacobian(f(x_s),x_s)
df_dx = @(x) [ 	0 , 		0  , 1 ;
				2*x(1) , 	-1 , 0 ;
				0 , 		sech(x(2))^2 , 0 ]

x = sdpvar(n,1);
th = sdpvar(p,1);

W00 = sdpvar(n,n,'full');
W01 = sdpvar(n,n,'full');
W10 = sdpvar(n,n,'full');
W02 = sdpvar(n,n,'full')'
W11 = sdpvar(n,n,'full')'
W20 = sdpvar(n,n,'full')'

W = W00 + ...
	W01 * x(1) + W10 * th(1) + ...
	W02 * x(1).^2 + W11 * x(1) * th(1) * W20 * th(1)^2;

temp_f = f(x);
W_dot = (W01 + W11 * th(1) + W02 * x(1)) * temp_f(1);

partialbi_W = 0;

nsd_dual_metric_expression = B0_perp' * ( W * ( df_dx(x)' ) + df_dx(x) * W - W_dot + 2 * lambda0 * W ) * B0_perp;
%zero_direction_dual_metric_constraint = [] %This appears to be zero.

degree(nsd_dual_metric_expression)

%[sol,v,Q,res] = solvesos(sos(-nsd_dual_metric_expression) );

out1 = optimize([-nsd_dual_metric_expression <= 0])