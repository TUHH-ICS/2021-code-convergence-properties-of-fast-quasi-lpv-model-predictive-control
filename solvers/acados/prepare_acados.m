%---------------------------------------------------------------------------------------------------
% For Paper
% "Convergence Properties of Fast quasi-LPV Model Predictive Control"
% by Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function ocp = prepare_acados(model, optim)
%PREPARE_ACADOS Prepare the acados optimizer for the respective model
%   This function sets up the acados optimizer for solving the optimal control problems required for
%   controlling the given model. Its configuration is left mostly standard, except where required
%   for levelling the playing field with qLMPC.
%
%   model -> Struct describing the model to implement a controller for
%   optim -> Struct with optimization parameters
%
%   ocp   -> Matlab object representing the prepared acados optimizer

% Calculate prediction horizon in seconds
ocp_N = double(optim.N);
ocp_T = ocp_N * model.dT;

%% Define cost function
% The cost function on ACADOS is defined in terms of a virtual output y. In
% out case, we stack the states and inputs together to form y.

nx = model.nx;
nu = model.nu;
ny = nx + nu;                   % Dimension of the virtual output
Vx = [ eye(nx); zeros(nu,nx) ]; % Mapping from states to virtual output
Vu = [ zeros(nx,nu); eye(nu) ]; % Mapping from inputs to virtual output

% Weighting matrix for the virtual output
W  = blkdiag(optim.Q, optim.R); 

%% Build ACADOS OCP model
% For details on the meaning of each of these terms, check the ACADOS documentation at 
% https://github.com/acados/acados/blob/master/docs/problem_formulation/problem_formulation_ocp_mex.pdf

ocp_model = acados_ocp_model();
ocp_model.set('name', model.name);
ocp_model.set('T', ocp_T);

% Set dimension of the decision variables
ocp_model.set('dim_nx', nx);
ocp_model.set('dim_nu', nu);
ocp_model.set('dim_ny', ny);
ocp_model.set('dim_ny_e', 0); % No additional terminal constraint. See also the comment further down
ocp_model.set('dim_nz', 0);   % No algebraic variables

% Set symbolic variables
sym_x = casadi.SX.sym('x', nx);
sym_u = casadi.SX.sym('u', nu);
ocp_model.set('sym_x', sym_x);
ocp_model.set('sym_u', sym_u);

% Define dynamic properties of the model
ocp_model.set('dyn_type', 'explicit'); % Use explicit continuous-time ODE as model
ocp_model.set('dyn_expr_f', model.ode(sym_x, sym_u));

% Set up cost function
ocp_model.set('cost_type', 'linear_ls');  % Linear least-squares cost function
ocp_model.set('cost_Vu', Vu);
ocp_model.set('cost_Vx', Vx);
ocp_model.set('cost_Vz', zeros(ny,0));    % No algebraic variables, so setting to 0
ocp_model.set('cost_W', W);
ocp_model.set('cost_y_ref', zeros(ny,1)); % Reference is at the origin, so setting to 0

% Use no additional terminal constraint. Note that Acados already penalizes the state at the final
% step using the running cost. This is not immediately clear from the problem formulation.
ocp_model.set('cost_type_e', 'linear_ls');
ocp_model.set('cost_Vx_e', []);
ocp_model.set('cost_W_e', []);
ocp_model.set('cost_y_ref_e', []);

% Temporarily set initial conditions, without this the model contruction fails. Will be overwritten
% before the simulation anyway.
ocp_model.set('constr_x0', model.x0);

%% Configure ACADOS OCP solver
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', ocp_N);

% Set up the internal simulator. Here Euler's is selected, as this is the integrator used for qLMPC
% and CasADi, to make for a fair comparison.
ocp_opts.set('sim_method', 'erk');        % Use explicit Runge-Kutta ODE solver
ocp_opts.set('sim_method_num_stages', 1); % Use 1th order ODE solver

% Set up the NLP solver. If the solver is only allowed to perform a single iteration, switch to
% real-time iteration mode.
if optim.iter_max == 1
    ocp_opts.set('nlp_solver', 'sqp_rti');
else
    ocp_opts.set('nlp_solver', 'sqp');
end
ocp_opts.set('nlp_solver_max_iter', optim.iter_max); % Maximum number of iterations

% Set up the QP solver
ocp_opts.set('qp_solver', 'full_condensing_hpipm');

%% Generate OCP
ocp = acados_ocp(ocp_model, ocp_opts);
end
