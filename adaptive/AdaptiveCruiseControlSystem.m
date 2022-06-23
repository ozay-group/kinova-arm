classdef AdaptiveCruiseControlSystem
    properties
        Mass;
        v0;
        f0;
        f1;
        f2;
        Options;
        % Dimensions of variables
        n_x;
        n_u;
        n_theta;
    end
    methods
        function acc = AdaptiveCruiseControlSystem(varargin)
            % Input Processing
            accOptions = ACCSystemArgumentParsing(varargin{:});

            % Algorithm
            acc.Options = accOptions;
            acc.Mass = accOptions.Mass;
            acc.f0 = accOptions.f0;
            acc.f1 = accOptions.f1;
            acc.f2 = accOptions.f2;
            acc.v0 = accOptions.v0;

            acc.n_x = 2;
            acc.n_u = 1;
            acc.n_theta = 3;
        end

        function F_Matrix = F(accSystem,x)
            %Description:
            %   The F(x) matrix in the dynamics of ACC.
            %   Recall the dynamics which are written as:
            %       x-dot = f(x) + F(x) theta^* + g(x) u
            
            % Constants
            v = x(1);

            % Algorithm
            F_Matrix = (1/accSystem.Mass) * [ 1 , v , v.^2 ; 0 , 0 , 0 ]; 
        end

        function Theta_star = Theta(accSystem)
            Theta_star = [ accSystem.f0 ; accSystem.f1 ; accSystem.f2 ];
        end

        function f_out = f(accSystem,x)
            %Description:
            %   The drift term of the dynamics of the ACC System.
            %   Recall the dynamics which are written as:
            %       x-dot = f(x) + F(x) theta^* + g(x) u

            % Constants
            v = x(1);

            % Algorithm
            f_out = [0; accSystem.v0 - v];
        end

        function g_out = g(accSystem,x)
            %Description:
            %   The affine input term of the dynamics of the ACC System.
            %   Recall the dynamics which are written as:
            %       x-dot = f(x) + F(x) theta^* + g(x) u

            % Constants

            % Algorithm
            g_out = [1/accSystem.Mass;0];
        end

        function dxdt = Dynamics(accSystem,x,u)
            %Description:
            %   Provides the full dynamics of the ACC System.
            %   Recall the dynamics which are written as:
            %       x-dot = f(x) + F(x) theta^* + g(x) u
            %
            %Assumptions:
            %   Assumes that the input is a scalar force input.

            % Constants

            % Algorithm
            dxdt = accSystem.f(x) + accSystem.F(x) * accSystem.Theta() + ...
                accSystem.g(x) * u;

        end
    end
end

function accOptions = ACCSystemArgumentParsing(varargin)
    %Description:
    %

    % Constants

    % Set Defaults
    accOptions = struct( ...
        'Mass',1650, ... %kg
        'f0',0.1, ... % ?
        'f1',5.0, ... % ?
        'f2',0.25, ... % ?
        'v0',15 ... % m/s
        );

    % Algorithm
    varargin_index = 1;
    while varargin_index <= nargin
        switch varargin{varargin_index}
            case 'Mass'
                accOptions.Mass = varargin{varargin_index+1};
                varargin_index = varargin_index + 2;
            case 'f0'
                accOptions.f0 = varargin{varargin_index+1};
                varargin_index = varargin_index + 2;
            case 'f1'
                accOptions.f1 = varargin{varargin_index+1};
                varargin_index = varargin_index + 2;
            case 'f2'
                accOptions.f2 = varargin{varargin_index+1};
                varargin_index = varargin_index + 2;
            case 'v0'
                accOptions.v0 = varargin{varargin_index+1};
                varargin_index = varargin_index + 2;
            otherwise
                error(['Unexpected input to AdaptiveCruiseControlSystem: ' varargin{varargin_index} ])
        end
    end

end