% test_slide_principle1.m
% Description:
%	Set up a simple sliding simulation where the variable we want to find is the initial slide parameters: 
%	- release velocity and 
%	- release position.

clear all; close all; clc;

%% Constants

m 	 = 0.075; 	% kg
mu_k = 0.25;
g 	 = 9.81; 	% kg

%% Simulate Linear System

x0 = [0;2];

tspan1 = 15;
[tout1,xout1] = ode45(@slide_dyn2,[0:tspan1],x0);

figure;
subplot(2,1,1)
plot(tout1,xout1(:,1))
xlabel('time (s)')
ylabel('position (m)')

subplot(2,1,2)
plot(tout1,xout1(:,2))
xlabel('time (s)')
ylabel('velocity (m/s)')

tspan2 = 15;
[tout2,xout2] = ode45(@slide_dyn1,[0:tspan2],x0);

figure;
subplot(2,1,1)
plot(tout1,xout2(:,1))
xlabel('time (s)')
ylabel('position (m)')
title('Position Trajectory with Mostly Smooth Dynamics')

subplot(2,1,2)
plot(tout1,xout2(:,2))
xlabel('time (s)')
ylabel('velocity (m/s)')


%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Function Definitions %%
%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_dot] = slide_dyn1(t,x)
	%Constants
	min_vel = 0.02;

	m 	 = 0.075; 	% kg
	mu_k = 0.25;
	g 	 = 9.81; 	% kg

	A = [0,1;0,0];

	% Calculate Derivative
	x_dot = A*x+[0;-1*sign(x(2))*mu_k*m*g];

end

function [x_dot] = slide_dyn2(t,x)
	%Constants
	min_vel = 0.02;

	m 	 = 0.075; 	% kg
	mu_k = 0.25;
	g 	 = 9.81; 	% kg

	A = [0,1;0,0];

	% Calculate Derivative

	if x(2) >= min_vel
		x_dot = A*x+[0;-1*sign(x(2))*mu_k*m*g];
	else
		x_dot = zeros(2,1);
	end

end