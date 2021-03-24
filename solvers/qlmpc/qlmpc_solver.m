function [u_opt, x_opt, stats] = qlmpc_solver(optim, model, nx, nu, x0, x, u) %#codegen
%QLMPC_SOLVER General solver for fast qLMPC
%   This function is a general solver for quasi-LPV MPC problems. It formulates the problem in a
%   dense way, such that the state variables are eliminated from the problem. The residual of the
%   Lagrangian gradient are used as the stopping criterion.
%
%   optim -> Struct with optimization parameters
%   model -> Function that return the LPV model of the plant
%   nx    -> Number of states
%   nu    -> Number of inputs
%   x0    -> Value of the state at the initial time
%   x     -> State trajectory used for warmstarting the solver
%   u     -> Input trajectory used for warmstarting the solver
%
%   u_opt -> Optimal control trajectory
%   x_opt -> Optimal state trajectory
%   stats -> Struct with timings and iteration information

N     = optim.N;
Q_hat = kron(eye(N+1), optim.Q);
R_hat = kron(eye(N),   optim.R);

% Initialize struct needed for dense problem formulation
Lambda = zeros((N+1)*nx, nx);
S      = zeros((N+1)*nx, N*nu);
Lambda(1:nx,1:nx) = eye(nx);

%% Initialize output values
u_opt = u;
x_opt = x;

stats = struct;
stats.iter = 0; % Solver iterations
stats.solv = 0; % Time spent solving the linear systems
stats.prep = 0; % Time spent preparing matrices

%% qLMPC iterations
% The solver is iterated until either optim.iter_max is hit, or the stopping criterion is satisfied
for i = 1:optim.iter_max
    %% Prepare the condensed problem formulation
    % For the condensed problem formulation, we prepare matrices to express the states as functions
    % of the control input, to eliminate the dynamic constraints.
    tic
    for k = 1:N
        % Calculate the system matrices of the LPV model at the current state and input
        x_range =  (k-1)*nx+1:k*nx;
        u_range =  (k-1)*nu+1:k*nu;
        [A, B] = model(x_opt(x_range), u_opt(u_range));

        % Update state equation at the current time
        Lambda(x_range+nx, 1:nx)    = A*Lambda(x_range, 1:nx);
        S(x_range+nx, u_range)      = B;
        for j = 1:k-1
            z_range =  (j-1)*nu+1:j*nu;
            S(x_range+nx, z_range)      = A*S(x_range, z_range);
        end
    end

    %% Solve quadratic program
    % Calculate QP cost function matrices
    c = S' * Q_hat * Lambda * x0;
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
        x_opt = Lambda*x0 + S*u_opt;
        stats.solv = stats.solv + toc;
        stats.iter = stats.iter + 1;
    end
end
end
