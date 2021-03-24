addpath('bench')
addpath('mex_stubs')
addpath(genpath('models'))
addpath(genpath('solvers'))

% Clear workspace for better repeatability. Clear mex is required because acados cannot overwrite
% its generated MEX-functions otherwise
clear
clear mex

setup_acados();

%% Script settings
% To speed up the qLMPC solvers, this script offers the possibility to compile the Matlab scripts
% into MEX functions. This flag controls whether this optimization is applied.
% If so, the generated MEX function will specialized to certain settings of the controller, such
% that it will need to be rebuilt every time this script is run.
generate_mex = true;

% Determines how many simulations should be performed for each algorithm for averaging purposes.
% This is mainly needed as the operating system is not deterministic.
averaging = 50;

% If set to true, then the results of the simulation including the benchmarking results will be
% exported from Matlab as csv files.
export = false;

%% Simulate the problem for each solver
[model, optim] = adip();
[x, u, solver_stats] = mpc_benchmark(model, optim, generate_mex, averaging);

% Evaluate the computational performance of the different solvers
evaluate_perf(model.name, optim, x, u, solver_stats, export);

%% Plot resulting ADIP trajectories
% Plot trajectory of each state individually
figure()
ax1 = subplot(221);
plot(0:model.dT:model.Tf, squeeze(x(1,1,:,:)))
xlabel('Time t in s')
ylabel('\theta_1(t)')
ax2 = subplot(222);
plot(0:model.dT:model.Tf, squeeze(x(2,1,:,:)))
xlabel('Time t in s')
ylabel('\theta_2(t)')
ax3 = subplot(223);
plot(0:model.dT:model.Tf, squeeze(x(3,1,:,:)))
xlabel('Time t in s')
ylabel('\omega_1(t)')
ax4 = subplot(224);
plot(0:model.dT:model.Tf, squeeze(x(4,1,:,:)))
xlabel('Time t in s')
ylabel('\omega_2(t)')
sgtitle('State Trajectories')
linkaxes([ax1, ax2,  ax3, ax4], 'x')
legend({solver_stats.name})

% Plot input trajectories
figure()
stairs(0:model.dT:model.Tf-model.dT, squeeze(u(1,1,:,:)))
xlabel('Time t in s')
ylabel('Torque \tau(t)')
title('Input Signal')
legend({solver_stats.name})
