%---------------------------------------------------------------------------------------------------
% For Paper
% "Convergence Properties of Fast quasi-LPV Model Predictive Control"
% by Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function [model, optim] = adip()
%ADIP Defines the arm-driven inverted pendulum model
%   This function defines the model for the arm-driven inverted pendulum. This means number of
%   states and inputs, but also initial condition and optimization parameters for the simulations.

nx = 4; % Number of states
nu = 1; % Number of inputs

%% Define simulation scenario
x0 = [ pi/3; 0; 0; 0 ]; % Initial condition
dT = 0.01;              % Sampling time of the controler and the underlying discrete-time system
Tf = 2;                 % Final simulation time of this scenario

%% Define solver and cost function parameters
N        = int32(40); % Prediction horizon in steps
iter_max = int32(30); % Maximum number of iterations before the solver should terminate
tol_stat = 1e-2;      % Tolerance on the stationarity gradient. If the gradient is less than this
                      % number, the qLMPC solver terminates

% We use a simple quadratic stage cost function in the form of
% V_k(x,u) = x(k)'*Q*x(k) + u(k)'*R*u(k)
Q = diag([200, 1000, 0.1, 10]);
R = 2000;

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
model.name = 'adip'; % It is important to use '' not "" here, otherwise model construction in ACADOS will fail
model.nx   = nx;
model.nu   = nu;
model.x0   = x0;
model.dT   = dT;
model.Tf   = Tf;

% For the subsequent functions, we include pointers to some functions belonging to this model, such
% that we don't need to use eval.
model.ode        = @adip_ode;
model.solver_lpv = @adip_solver;
model.solver_vel = @adip_solver_vel;
end
