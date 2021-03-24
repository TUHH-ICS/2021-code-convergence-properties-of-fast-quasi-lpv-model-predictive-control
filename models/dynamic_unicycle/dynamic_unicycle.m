%---------------------------------------------------------------------------------------------------
% For Paper
% "Convergence Properties of Fast quasi-LPV Model Predictive Control"
% by Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function [model, optim] = dynamic_unicycle()
%DYNAMIC_UNICYCLE Defines the dynamic unicycle model
%   This function defines the model for the dynamic unicycle. This means number of states and
%   inputs, but also initial condition and optimization parameters for the simulations.

nx = 5; % Number of states
nu = 2; % Number of inputs
     
%% Define simulation scenario
x0 = [ 1; 2; 0; pi; 0 ]; % Initial condition
dT = 0.1;                % Sampling time of the controler and the underlying discrete-time system
Tf = 10;                 % Final simulation time of this scenario

%% Define solver and cost function parameters
N        = int32(20); % Prediction horizon in steps
iter_max = int32(1);  % Maximum number of iterations before the solver should terminate
tol_stat = 1e-5;      % Tolerance on the stationarity gradient. If the gradient is less than this
                      % number, the qLMPC solver terminates

% We use a simple quadratic stage cost function in the form of
% V_k(x,u) = x(k)'*Q*x(k) + u(k)'*R*u(k)
Q = diag([1, 1, 0.1, 1, 0.1]);
R = diag([1, 1]);

%% Collect data into structures

% Collect in optimizer settings
optim = struct;
optim.N        = N;
optim.Q        = Q;
optim.R        = R;
optim.iter_max = iter_max;
optim.tol_stat = tol_stat;

% Collect all other model specific values
model      = struct;
model.name = 'dynamic_unicycle'; % It is important to use '' not "" here, otherwise model construction in ACADOS will fail
model.nx   = nx;
model.nu   = nu;
model.x0   = x0;
model.dT   = dT;
model.Tf   = Tf;

% For the subsequent functions, we include pointers to some functions belonging to this model, such
% that we don't need to use eval.
model.ode        = @dynamic_unicycle_ode;
model.solver_lpv = @dynamic_unicycle_solver;
model.solver_vel = @dynamic_unicycle_solver_vel;
end
