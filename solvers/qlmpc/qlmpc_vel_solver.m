%---------------------------------------------------------------------------------------------------
% For Paper
% "Convergence Properties of Fast quasi-LPV Model Predictive Control"
% by Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function [u_opt, x_opt, stats] = qlmpc_vel_solver(optim, model, nx, nu, nd, x0, x, u) %#codegen
%QLMPC_VEL_SOLVER General solver for fast qLMPC in exact formulation
%   This function is a general solver for quasi-LPV MPC problems. It formulates the problem in a
%   dense way, such that the state variables are eliminated from the problem. The residual of the
%   Lagrangian gradient are used as the stopping criterion. This solver includes a disturbance model
%   to allow calculating the true optimal value.
%
%   optim -> Struct with optimization parameters
%   model -> Function that return the LPV model of the plant
%   nx    -> Number of states
%   nu    -> Number of inputs
%   nd    -> Number of disturbance inputs
%   x0    -> Value of the state at the initial time
%   x     -> State trajectory used for warmstarting the solver
%   u     -> Input trajectory used for warmstarting the solver
%
%   u_opt -> Optimal control trajectory
%   x_opt -> Optimal state trajectory
%   stats -> Struct with timings and iteration information

tval = tic;

%% Initialize solver variables
N     = optim.N;
Q_hat = kron(eye(N+1), optim.Q);
R_hat = kron(eye(N),   optim.R);

% The initial state is augmented by the disturbance model, therefore we need to allocate more space
x_hat  = zeros(nx+N*nd,  1);
x_hat(1:nx) = x0;

% Initialize struct needed for dense problem formulation
Lambda = zeros((N+1)*nx, nx+N*nd);
S      = zeros((N+1)*nx, N*nu);
Lambda(1:nx,1:nx) = eye(nx);

%% Initialize output values
u_opt = u;
x_opt = x;

stats = struct;
stats.iter  = 0; % Solver iterations
stats.solv  = 0; % Time spent solving the linear systems
stats.prep  = 0; % Time spent preparing matrices
stats.total = 0; % Total solver time

%% qLMPC iterations
for i = 1:optim.iter_max
    %% Prepare the condensed problem formulation
    tic
    for k = 1:N
        % Calculate the system matrices of the LPV model at the current state and input
        x_range =  (k-1)*nx+1:k*nx;
        u_range =  (k-1)*nu+1:k*nu;
        d_range = ((k-1)*nd+1:k*nd)+nx;
        [A, B, Bd, d] = model(x_opt(x_range), u_opt(u_range));

        % Update state equation at the current time. For the exact qLMPC variant that is implemented
        % here, this also includes the disturbance model.
        x_hat(d_range) = d;
        Lambda(x_range+nx, 1:nx)    = A*Lambda(x_range, 1:nx);
        Lambda(x_range+nx, d_range) = Bd;
        S(x_range+nx, u_range)      = B;
        for j = 1:k-1
            y_range = ((j-1)*nd+1:j*nd)+nx;
            z_range =  (j-1)*nu+1:j*nu;

            Lambda(x_range+nx, y_range) = A*Lambda(x_range, y_range);
            S(x_range+nx, z_range)      = A*S(x_range, z_range);
        end
    end

    %% Solve quadratic program
    % Calculate QP cost function matrices
    c = S' * Q_hat * Lambda * x_hat;
    H = S' * Q_hat * S + R_hat;

    stats.prep = stats.prep + toc;
    
    % Calculate infinity norm of the Lagrangian gradient and check whether it is below the tolerance
    % level.
    if norm(c + H*u_opt, inf) < optim.tol_stat
        break
    else
        % Solve the linear system that represents the KKT-conditions of the QP
        tic
        u_opt = -H\c;
        x_opt = Lambda*x_hat + S*u_opt;
        stats.solv = stats.solv + toc;
        stats.iter = stats.iter + 1;
    end
end

stats.total = toc(tval); % Save total solver time

end
