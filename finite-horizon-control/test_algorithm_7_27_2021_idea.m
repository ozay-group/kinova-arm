%test_algorithm_7_27_2021_idea.m
%Description:
%	Testing the CPre() algorithm idea that Necmiye developed.

%% Cleanup
clear all; close all; clc;

%% Add pcis to path
addpath(genpath('~/Documents/Michigan/Research/pcis/'))

%% Create Simple Systems

n_x = 1;
n_u = 1;

A = eye(n_x);
B = 1*eye(n_u);

eta_u = 1.0;
eta_w = 0.1;

XU = Polyhedron('A',[ zeros(2,1) [1;-1] ],'b',eta_u*ones(2,1));
W1 = Polyhedron('lb',-eta_w,'ub',eta_w);

d1 = Dyn(	A,0,B,XU,...
			{},{}, Polyhedron(), ...
			{1},{0},W1);

d1.check(); %Check to make sure that everything is correctly made.

W2 = W1 + eta_w;
d2 = Dyn( 	A , 0 , B , XU , ...
			{},{}, Polyhedron(), ...
			{1} , {0} , W2 );
d2.check();

%% Create Target(s)
XT = Polyhedron('lb',3,'ub',4);

T = 3;

% Calculate First Few Pre''s
CPre_sys1 = {};
CPre_sys2 = {};

CPre_sys1{1} = XT; CPre_sys2{1} = XT;

for tau = 1:T
	CPre_sys1{1+tau} = d1.pre(CPre_sys1{tau});
	CPre_sys2{1+tau} = d2.pre(CPre_sys2{tau});
end

% Plot The Pre's According to Each
figure;

subplot(1,2,1)
hold on;
for tau = 0:T
	plot( Polyhedron('lb',tau,'ub',tau) * CPre_sys1{tau+1} ) %X_T
end
xlabel('Time Index $k$','Interpreter','latex')
ylabel('State Value $x(k)$','Interpreter','latex')
axis([-0.1,3.1,0,5.7])

subplot(1,2,2)
hold on;
for tau = 0:T
	plot( Polyhedron('lb',tau,'ub',tau) * CPre_sys2{tau+1} ) %X_T
end
xlabel('Time Index $k$','Interpreter','latex')
ylabel('State Value $x(k)$','Interpreter','latex')
axis([-0.1,3.1,0,5.7])

%% Compute PreSets using pre_int
Cpre_12 = {};
Cpre_12{1} = XT;

for tau = 1:T
	Cpre_12{1+tau} = pre_int(d1,d2,Cpre_12{tau},0);
end

figure;
hold on;

for tau = 0:T
	plot( Polyhedron('lb',tau,'ub',tau) * Cpre_12{tau+1} )
end

xlabel('Time Index $k$','Interpreter','latex')
ylabel('State Value $x(k)$','Interpreter','latex')
axis([-0.1,3.1,0,5.7])

%%%%%%%%%%%%%%%
%% Functions %%
%%%%%%%%%%%%%%%

function [preSets,preXU] = pre_int( dyn1 , dyn2 , X , rho )
	%Description:
	%	This function computes the set of states from which a control exists that will lead the state to the target set
	%	X regardless of if the dynamics dyn1 or the dynamics dyn2 are active.

	%% Input Processing %%
	if isempty(rho)
		rho = 0;
	end

	%% Constants %%
	n_x = dyn1.nx();

	%% Algorithm %%
	preXU_i = {};
	preXU_i{1} = dyn1.pre_xu(X,rho);
	preXU_i{2} = dyn2.pre_xu(X,rho);
	figure; subplot(211); plot(preXU_i{1}); subplot(212); plot(preXU_i{2});

	preXU = preXU_i{1}.intersect(preXU_i{2});

	preSets = preXU.projection([1:n_x]);

end