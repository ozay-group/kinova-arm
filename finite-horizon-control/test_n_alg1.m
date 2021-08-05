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

% Create the Initial Set in Belief Space
XT_Belief = { XT , XT , XT }; %XT_Belief{1} is for belief 1, XT_Belief{2} is for belief 2, and XT_Belief{3} is for belief {1,2}

Xpre1 = pre_belief_space( d1 , d2 , XT_Belief , 0 );

%%%%%%%%%%%%%%%
%% Functions %%
%%%%%%%%%%%%%%%

function [ Xpre_Belief ] = pre_belief_space( dyn1 , dyn2 , X_Belief , rho )
	%Description:
	%	

	%% Constants
	X_1 = X_Belief{1}; X_2 = X_Belief{2}; X_12 = X_Belief{3};
	n_w = dyn1.nw(); n_x = dyn1.nx(); n_u = dyn1.nu();

	eta0 = 10^(-2);

	Xpre_Belief = {};

	%% Pre For Region with Hypotheses {1} and {2}.
	Xpre_Belief{1} = dyn1.pre(X_1,rho);
	Xpre_Belief{2} = dyn2.pre(X_2,rho);

	%% Pre With Necmiye's Experimental Pre
	Xpre_Belief{3} = pre_int( dyn1 , dyn2 , X_12 , rho );

	% The transition pre
	% H = [ 	dyn1.XU.A , zeros( size(dyn1.XU.A,1) , 2*n_w) ;
	% 		dyn2.XU.A , zeros( size(dyn2.XU.A,1) , 2*n_w) ;
	% 		dyn1.A - dyn2.A , dyn1.B - dyn2.B , 1 , -1 ;
	% 		-[dyn1.A - dyn2.A , dyn1.B - dyn2.B , 1 , -1] ];
	% h = [ ... 
	% 	dyn1.XU.b ; 
	% 	dyn2.XU.b ;
	% 	eta0;
	% 	-eta0
	% 	]

	H0 = [ 	dyn1.XU.A ;
			dyn2.XU.A ];

	h0 = [ 	dyn1.XU.b ; 
			dyn2.XU.b ];

	DV1 =  dyn1.D.V; DV2 = dyn2.D.V;
	for v1_index = 1:size(DV1,1)
		for v2_index = 1:size(DV2,1)

			temp_v1 = DV1(v1_index,:)';
			temp_v2 = DV2(v2_index,:)';

			% Create Constraint for each vertex.
			Hi = [ 	H0;
					dyn1.A - dyn2.A , dyn1.B - dyn2.B ];
			hi = [	h0;
					eta0 - (temp_v1 - temp_v2) ];

			temp_Pxu = Polyhedron('A',Hi,'b',hi);
			temp_Pre_Component = temp_Pxu.projection([1:n_x]); 

			if temp_Pre_Component.isEmptySet
				disp(['Empty with +: ' num2str(v1_index) ' x ' num2str(v2_index) ])
			end

			% Create the set of points which work for greater than or equal.

			Hi = [ 	H0;
					-[dyn1.A - dyn2.A , dyn1.B - dyn2.B] ];
			hi = [	h0;
					- eta0 + (temp_v1 - temp_v2) ];

			temp_Pxu = Polyhedron('A',Hi,'b',hi);
			temp_Pre_Component = temp_Pxu.projection([1:n_x]);

			if temp_Pre_Component.isEmptySet
				disp(['Empty with -: ' num2str(v1_index) ' x ' num2str(v2_index) ])
			end

		end
	end

end



function [ preSets , preXU ] = pre_int( dyn1 , dyn2 , X , rho )
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
	%figure; subplot(211); plot(preXU_i{1}); subplot(212); plot(preXU_i{2});

	preXU = preXU_i{1}.intersect(preXU_i{2});

	preSets = preXU.projection([1:n_x]);

end