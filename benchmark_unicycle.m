%---------------------------------------------------------------------------------------------------
% For Paper
% "Convergence Properties of Fast quasi-LPV Model Predictive Control"
% by Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

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
[model, optim] = dynamic_unicycle();
[x, u, solver_stats] = mpc_benchmark(model, optim, generate_mex, averaging);

% Evaluate the computational performance of the different solvers
evaluate_perf(model.name, optim, x, u, solver_stats, export);

%% Export unicycle trajectory
pos_s = squeeze(x(1,1,:,:));
pos_q = squeeze(x(2,1,:,:));

% Optionally export the unicycle trajectory in the sq-plane into a CSV file
if export
    writetable(array2table([(0:model.dT:model.Tf)', pos_s, pos_q], 'VariableNames',...
               ['time', strcat('x_', {solver_stats.name}), strcat('y_', {solver_stats.name})]),...
               'dynamic_unicycle_traj.csv')
end

%% Plot resulting unicycle trajectories
% Plot position trajectory in the sq-plane
figure()
plot(pos_s, pos_q)
title('Trajectory of the Unicycle')
xlabel('Coordinate s(t)')
ylabel('Coordinate q(t)')
legend({solver_stats.name}, 'Location', 'northwest')

% Plot trajectory of each state individually
figure()
ax1 = subplot(511);
plot(0:model.dT:model.Tf, squeeze(x(1,1,:,:)))
xlabel('Time t in s')
ylabel('s(t)')
ax2 = subplot(512);
plot(0:model.dT:model.Tf, squeeze(x(2,1,:,:)))
xlabel('Time t in s')
ylabel('q(t)')
ax3 = subplot(513);
plot(0:model.dT:model.Tf, squeeze(x(3,1,:,:)))
xlabel('Time t in s')
ylabel('v(t)')
ax4 = subplot(514);
plot(0:model.dT:model.Tf, squeeze(x(4,1,:,:)))
xlabel('Time t in s')
ylabel('\Phi(t)')
ax5 = subplot(515);
plot(0:model.dT:model.Tf, squeeze(x(5,1,:,:)))
xlabel('Time t in s')
ylabel('\omega(t)')
sgtitle('State Trajectories')
linkaxes([ax1, ax2, ax3, ax4, ax5], 'x')
legend({solver_stats.name})

% Plot input trajectories
figure()
ax1 = subplot(211);
stairs(0:model.dT:model.Tf-model.dT, squeeze(u(1,1,:,:)))
xlabel('Time t in s')
ylabel('Force F(t)')
ax2 = subplot(212);
stairs(0:model.dT:model.Tf-model.dT, squeeze(u(2,1,:,:)))
xlabel('Time t in s')
ylabel('Torque \tau(t)')
sgtitle('Input Signals')
linkaxes([ax1, ax2], 'x')
legend({solver_stats.name})
