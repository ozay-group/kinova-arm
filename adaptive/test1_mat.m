%test1_mat.m
% Description:


clear all; close all; clc;

%% Main Script

% constants
x0 = [0;10];
tspan1 = [0,10];

% test ode45 once
[tout1,yout1] = ode45( ...
    @(t,x) accDynamics(t,x,0,[0.1,5.0,0.25,15]'), ...
    tspan1, ...
    x0);

figure;
plot(tout1,yout1)
title(['Zero Input ACC System with x0 = ' num2str(x0')] )

% test ode45 with closed loop behavior
tspan2 = [0,20];
[tout2,yout2] = ode45( ...
    @(t,x) cl_acc1(t,x,[0.1,5.0,0.25,NaN]'), ...
    tspan2, ...
    x0);

figure;

plot(tout2,yout2)
legend('v','D')
title(['CL ACC System with x0 = ' num2str(x0')] )

% test ode45 with closed loop behavior ON Class
acc0 = AdaptiveCruiseControlSystem();
tspan3 = [0,20];
[tout3,yout3] = ode45( ...
    @(t,x) acc0.Dynamics(x,acc0.Mass*2.0), ...
    tspan3, ...
    x0);

figure;
%
plot(tout3,yout3)
legend('v','D')
title(['CL ACC System with x0 = ' num2str(x0') ' Using Class Definition'] )

thetaHat0 = [acc0.f0-0.1,acc0.f1-0.05,acc0.f2-0.05]';
% h_a = alpha^2 - (D - 1.8*v - alpha)^2;
alpha0 = 12;
h_a0 = @(x,thetaHat) alpha0^2 - (x(2) - 1.8 * x(1) - alpha0)^2;
acc2_aCBF_controller(x0,thetaHat0,h_a0,acc0)

%% Class Definitions

%% Functions

function [x_dot] = accDynamics(t,x,u,param)
    % constants
    v = x(1);
    D = x(2);

    m = 1650; % kg

    if isempty(param)
        param = [0.1,5.0,0.25,15]';
    end

    theta = param([1:3]);

    v0 = param(4); % m/s

    % algorithm

    x_dot = [ 0 ; v0 - v ] - (1/m) * [ 1,v,v^2 ; 0,0,0] * theta + [1/m;0] * u;

end

function [x_dot] = cl_acc1(t,x,params)
    % constants
    K_v = -10^2;
    K_D = 3*10^2;
    v0 = 15; % m/s
    m = 1650; % kg

    a_max = 10; %m/s^2
    a_min = -5; %m/s^2

    v_des = v0 + 5;
    D_des = 5; % meters

    % algorithm
    v = x(1);
    D = x(2);

    if length(params) < 4
        params(4) = v0;
    elseif isnan(params(4))
        params(4) = v0;
    end

    a_K = K_v*(v - v_des) + K_D * (D - D_des);
    a_clipped = max( a_min, min(a_K,a_max));

    u_clipped = m * a_clipped;

    x_dot = accDynamics(t,x,u_clipped,params);
end


function acc2_aCBF_controller(x,thetaHat,h_a,accSystem)
    %acc2_aCBF_controller
    %Description:
    %   Uses the closed loop adaptive cruise control system based on the
    %   aCBF controller in Taylor's paper.
    %
    %Inputs:
    %   x = Current State of The System (2x1 vector)
    %   thetaHat = Current Estimate of the Parameter of The System from (3x1 vector)
    %   h_a = Function which receives as input two different values x, thetaHat (function of x, thetaHat)
    

    % Constants
    
    n = 2; % System dimension. This is the case for accSystem
    m = 1; % Input dimension; this is 
    p = 3; % Parameter dimension

    Gamma = eye(p);


    kp_t = [1,0] * x + [0,0.1,0]*thetaHat;

    % Algorithm

    % u comes from a QP
    u = sdpvar(m,1);

    % Compute gradient of h_a
    x_s = sym('x',[n,1])
    thetaHat_s = sym('thetaHat',[p,1]);
    h_a_t = h_a(x_s,thetaHat_s);

    dha_dx = gradient(h_a_t,x_s)
    dha_dx_t = subs(dha_dx,struct('x1',x(1),'x2',x(2)));
    dha_dx_t = double(dha_dx_t); %Transform this from type sym to a double
    
    %dha_dh is zero here
    dha_dth = gradient(h_a_t,thetaHat_s);
    dha_dth_t = subs(dha_dth,struct('thetaHat1',thetaHat(1),'thetaHat2',thetaHat(2),'thetaHat3',thetaHat(3)));
    dha_dth_t = double( dha_dth_t ); %Transform this from type sym to a double

    lambda_cbf_t = thetaHat - Gamma * dha_dth_t;

    ( accSystem.f(x) + accSystem.F(x) * ( thetaHat + Gamma * lambda_cbf_t ) + accSystem.g(x) * u )

    gradient_condition_constraint = [ dha_dx_t' * ( accSystem.f(x) + accSystem.F(x) * ( thetaHat + Gamma * lambda_cbf_t ) + accSystem.g(x) * u ) >= 0 ];

    % Create objective
    objective = [ 0.5* norm( u - kp_t , 2 ) ];

    % Optimize
    ops = sdpsettings('verbose',1);
    opt0 = optimize(gradient_condition_constraint,objective,ops);

    disp(value(u))

end