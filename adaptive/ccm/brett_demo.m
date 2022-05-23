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
lambda1 = sdpvar(1,1,'full');

B0 = B(x0);
B0_perp = null(B0');

df_dx = jacobian(f(x_s),x_s);
% df_dx = @(x) [ 	0 , 		0  , 1 ;
% 				2*x(1) , 	-1 , 0 ;
% 				0 , 		sech(x(2))^2 , 0 ]
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

nsd_dual_metric_expression = B0_perp' * ( W * ( df_dx(x)' ) + df_dx(x) * W - W_dot + 2 * lambda1 * W ) * B0_perp;
%zero_direction_dual_metric_constraint = [] %This appears to be zero.

degree(nsd_dual_metric_expression)

%[sol,v,Q,res] = solvesos(sos(-nsd_dual_metric_expression) );

disp('Solving Metric Synthesis problem...')
out1 = optimize([nsd_dual_metric_expression <= 0,W00 >= 0, W01 >= 0, W10 >= 0, W02 >= 0,W11 >= 0, W20 >= 0])

%% Plotting Some Trajectories %%

M = @(x,th) ( value(W00) + value(W01) * x(1) + value(W10) * th(1) + value(W02) * x(1).^2 + value(W11) * x(1) * th(1) * value(W20) * th(1)^2 )^-1;

disp('Plotting some data.')
%Choose some initial conditions.
theta0 = [0.05, 0.05, 0.05]';
x0_1 = [0.1,0,0.1]'; x0_2 = [0,0.1,0]';
tspan1 = [0:0.001:10];
tspan2 = tspan1;


[tout1, yout1] = ode45(@(t,x) dx_dt(x,theta0,0), tspan1 , x0_1);
[tout2, yout2] = ode45(@(t,x) dx_dt(x,theta0,0), tspan2 , x0_2);

figure;
subplot(2,1,1)
plot(tout1,yout1)

subplot(2,1,2)
plot(tout2,yout2)

figure;
yout_diff = yout1 - yout2;
for idx1 = [1:length(tout1)]
	metric_val(idx1) = yout_diff(idx1,:) * M(yout_diff(idx1,:),theta0) * yout_diff(idx1,:)';
end
plot(tout1,metric_val)
