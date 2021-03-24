function [x_traj, u_traj, solver_stats] = mpc_benchmark(model, optim, generate_mex, averaging)
%MPC_BENCHMARK Run the benchmark scenario for the different solvers
%   This function runs the benchmark scenario that is defined by model & optim for the different MPC
%   solvers. Optionally, you can request that it builds optimized MEX-functions for the qLMPC
%   solvers or runs the simulation multiple times in order to perform averaging afterwards.

solvers = {'qlmpc', 'vqlmpc', 'acados', 'casadi'};
n_solvers = length(solvers);

%% Build specialized solvers if requested
% If requested by setting generate_mex, at this point the specialized solvers will be constructed.
% They are specialized for the selected values of N, Q and R. In this way the C compiler can apply
% more optimizations, resulting in potentially faster solvers.
if generate_mex
    build_solver_mex(model, optim)
    
    % These two lines are a workaround for a seemingly unavoidable shortcomming of Matlab. The
    % MEX-functions that were generated in build_solver_mex cannot be called otherwise from this
    % function.
    ocp_qlmpc = @ocp_qlmpc;
    ocp_qlmpc_vel = @ocp_qlmpc_vel;
else
    warning('Running interpreted Matlab code for qLMPC solvers. This will be comparatively slow!')
    
    % Shim the MEX functions with the existing interpreted Matlab functions. In this way, the
    % remaining code can be run unchanged compared to the optimized version.
    ocp_qlmpc     = model.solver_lpv;
    ocp_qlmpc_vel = model.solver_vel;
end

% Load function into memory, to make the timing of the first call realistic
ocp_qlmpc(    optim, model.x0, zeros((optim.N+1)*model.nx,1), zeros(optim.N*model.nu,1));
ocp_qlmpc_vel(optim, model.x0, zeros((optim.N+1)*model.nx,1), zeros(optim.N*model.nu,1));

% Prepare solvers from acados and CasADi / Ipopt
ocp_acados = prepare_acados(model, optim);
ocp_casadi = prepare_casadi(model, optim);

%% Reserve memory for saving the simulation results
steps = floor(model.Tf/model.dT);

% Reserve memory for simulated trajectories
x_traj          = zeros(model.nx, averaging, steps+1, n_solvers);
x_traj(:,:,1,:) = repmat(model.x0, 1, averaging, 1, n_solvers);
u_traj          = zeros(model.nu, averaging, steps, n_solvers);

% Prepare time statistics
solver_stats = struct('name',  solvers,...
                      'total', zeros(averaging, steps),...
                      'solv',  zeros(averaging, steps),...
                      'prep',  zeros(averaging, steps),...
                      'iter',  zeros(averaging, steps));

%% Simulate the problem for each solver
% This section contains multiple nested loop, as the solvers must be called repeatedly for these
% benchmark scenarios. The outmost loop performs the (optional) averaging, the second runs the
% simulation for each of the solvers and the innermost iterates over the duration of the scenario.

for j = 1:averaging
    for i = 1:n_solvers
        % Initialize variables for warm starting the solvers
        x = kron(ones(optim.N+1,1), model.x0);
        u = zeros(optim.N*model.nu, 1);

        for k = 1:steps
            switch i
                case 1
                    % Run the standard qLMPC solver
                    tmval = tic;
                    [u, x, stats] = ocp_qlmpc(optim, x_traj(:,j,k,i), x, u);
                    solver_stats(i).total(j,k) = toc(tmval);
                    solver_stats(i).iter(j,k)  = stats.iter;
                    solver_stats(i).prep(j,k)  = stats.prep;
                    solver_stats(i).solv(j,k)  = stats.solv;
                    u_traj(:,j,k,i)            = u(1:model.nu);
                case 2
                    % Run the exact variant of qLMPC
                    tmval = tic;
                    [u, x, stats] = ocp_qlmpc_vel(optim, x_traj(:,j,k,i), x, u);
                    solver_stats(i).total(j,k) = toc(tmval);
                    solver_stats(i).iter(j,k)  = stats.iter;
                    solver_stats(i).prep(j,k)  = stats.prep;
                    solver_stats(i).solv(j,k)  = stats.solv;
                    u_traj(:,j,k,i)            = u(1:model.nu);
                case 3
                    % Run the acados based solver
                    ocp_acados.set('constr_x0', x_traj(:,j,k,i));
                    tmval = tic;
                    ocp_acados.solve();
                    solver_stats(i).total(j,k) = toc(tmval);
                    solver_stats(i).iter(j,k)  = ocp_acados.get('sqp_iter');
                    solver_stats(i).prep(j,k)  = ocp_acados.get('time_lin')...
                            + ocp_acados.get('time_reg') + ocp_acados.get('time_sim');
                    solver_stats(i).solv(j,k)  = ocp_acados.get('time_qp_sol');
                    u_traj(:,j,k,i) = ocp_acados.get('u', 0);
                case 4
                    % Run the CasADi / Ipopt solver
                    tmval = tic;
                    sol = ocp_casadi('p', x_traj(:,j,k,i), 'x0', u);
                    solver_stats(i).total(j,k) = toc(tmval);
                    u_traj(:,j,k,i) = full(sol.x(1:model.nu));
                    u = sol.x;
            end

            % Simulate continuous-time system
            odeFun = @(~,y) model.ode(y, u_traj(:,j,k,i));
            [~,y] = ode45(odeFun, [0, model.dT]+(k-1)*model.dT, x_traj(:,j,k,i));
            x_traj(:,j,k+1,i) = y(end, :);
        end
    end
end
end
