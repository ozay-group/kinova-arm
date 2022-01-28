%convert_cbc_script1.m
%Description:
%   Converts the elements of a ConsistentBeliefController into a format
%   that can be easily transformed into Python code.

%% Constants


%% Algorithm
K_set = toy2_controller.K_set;
k_set = toy2_controller.k_set;

AMatrices = {}; BMatrices = {};
CMatrices = {}; fMatrices = {};
W_HMatrices = {}; W_hMatrices = {};
for dyn_index = 1:length(toy2_controller.System.Dyn)
    temp_dyn = toy2_controller.System.Dyn(dyn_index);
    AMatrices{dyn_index} = temp_dyn.A;
    BMatrices{dyn_index} = temp_dyn.B;
    CMatrices{dyn_index} = temp_dyn.C;
    fMatrices{dyn_index} = temp_dyn.f;

    W_HMatrices{dyn_index} = temp_dyn.P_w.A;
    W_hMatrices{dyn_index} = temp_dyn.P_w.b;
    % Assume that there are no equality constraints in W.
    x0 = toy2_controller.System.X0.V';
end

% Save language and input set
Lwords = toy2_controller.System.L.words;
U_H = toy2_controller.System.U.A;
U_h = toy2_controller.System.U.b;

save('test_cbc1.mat',...
    'AMatrices', 'BMatrices', 'CMatrices', 'fMatrices', ...
    'x0', 'Lwords' , 'U_H' , 'U_h', ...
    'K_set','k_set')
%% Functions
 