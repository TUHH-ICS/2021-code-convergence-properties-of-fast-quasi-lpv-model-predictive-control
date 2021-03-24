%---------------------------------------------------------------------------------------------------
% For Paper
% "Convergence Properties of Fast quasi-LPV Model Predictive Control"
% by Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function ocp = prepare_casadi(model, optim)
%PREPARE_CASADI Prepare the optimal control problem using CasADi
%   This function uses CasADi to formulate the optimal control problem defined in model & optim. The
%   return value can be used to solve it repeatedly afterwards.
%
%   model -> Struct describing the model to implement a controller for
%   optim -> Struct with optimization parameters
%
%   ocp   -> CasADi function solving the optimal control problem for the given model

% Initialize empty nonlinear program
X0 = casadi.SX.sym('x0', model.nx);
Xk = X0; % Parameter for the initial state
J  = 0;  % Cost function
w  = {}; % Collection of the optimization variables

% Construct optimal control problem in single-shooting formulation
for k = 1:optim.N
    Uk = casadi.SX.sym(['U_' num2str(k-1)], model.nu);
    w  = [w(:)', {Uk}];
    J  = J + Xk'*optim.Q*Xk + Uk'*optim.R*Uk;
    
    % Use Euler discretization for the shooting formulation
    Xk = Xk + model.dT * model.ode(Xk, Uk);
end

% Add penalty to the terminal state
J  = J + Xk'*optim.Q*Xk;

% Set options to prevent IPOPT and CasADi from printing debug information
ip   = struct('print_level', 0); 
opts = struct('ipopt', ip, 'print_time', false);

% Construct parametric nonlinear program
nlp  = struct('f', J, 'x', vertcat(w{:}), 'p', {X0});
ocp  = casadi.nlpsol('OCP_Casadi', 'ipopt', nlp, opts);
end
