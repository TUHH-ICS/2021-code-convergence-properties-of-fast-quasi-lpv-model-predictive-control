function evaluate_perf(name, optim, x_traj, u_traj, solver_stats, export)
%EVALUATE_PERF Evaluate the computational performance of the different MPC solvers
%   This function calculates performance measures for the different MPC solvers and prints them to
%   the console. This includes median, min and max solver times as well as the relative cummulative
%   suboptimality.
%
%   name         -> Name of the model
%   optim        -> Struct with optimization parameters, especially Q & R
%   x_traj       -> State trajectories
%   u_traj       -> Input trajectories
%   solver_stats -> Solver statistics struct, includes timings and iterations
%   export       -> If true, the tables are exported to CSV files

solvers   = {solver_stats.name};
n_solvers = length(solvers);
steps     = size(solver_stats(1).total, 2);

%% Intermediate calculations
% If we performed multiple rounds of simulation for averaging purposes, we now average the values
% out.  Also, convert from seconds to milliseconds.
mean_total = squeeze(mean(cat(3, solver_stats.total), 1)) * 1e3;
mean_solv  = squeeze(mean(cat(3, solver_stats.solv),  1)) * 1e3;
mean_prep  = squeeze(mean(cat(3, solver_stats.prep),  1)) * 1e3;
mean_iter  = squeeze(mean(cat(3, solver_stats.iter),  1));

% Calculate the distance to reference at the final simulation step for all solvers
total_cost = zeros(n_solvers, 1);
for i = 1:n_solvers
    for k = 1:steps
        % Regardless of averaging, always calculate this measure for the first simulation that was
        % performed
        x = squeeze(x_traj(:,1,k,i));
        u = squeeze(u_traj(:,1,k,i));
        
        total_cost(i) = total_cost(i) + x'*optim.Q*x + u'*optim.R*u;
    end
end

%% Assemble that highlight different aspects of the performance
% The first table contains median, min and max values for the total time of the solvers, as well as
% the RCSO. As Ipopt is used as the reference for the RCSO, don't include this value for that solver
table_perf = table(median(mean_total)', min(mean_total)', max(mean_total)',...
                   (total_cost - total_cost(4))/total_cost(4),...
                   'VariableNames', {'median', 'min', 'max', 'rcso'},...
                   'RowNames', solvers);
table_perf{4, 4} = NaN; % Ipopt is the reference

% The second table contains timings for different subphases of the solvers. Acados does not return
% valid results for timings < 1 millisecond, therefore the values are ignored.
table_subtask = table(median(mean_total)', median(mean_solv)', median(mean_prep)',...
                      (total_cost - total_cost(4))/total_cost(4),...
                      'VariableNames', {'total', 'qp', 'prep', 'rcso'},...
                      'RowNames', solvers);
table_subtask{3:4, 2} = NaN; % Ipopt and acados do not return valid results
table_subtask{3:4, 3} = NaN; % Ipopt and acados do not return valid results
table_subtask{4, 4}   = NaN; % Ipopt is the reference

%% Plot solver times
% Plot the total solver time, averaged over all simulations that were performed for this purpose
figure()
semilogy(1:steps, mean_total)
title('Solver Performance')
xlabel('Step k')
ylabel('Solver time in ms')
xlim([1, steps]);
legend(solvers)

% If the solvers were allowed to use more than one iteration, also plot the iterations the solvers
% took at each step of the simulation
if optim.iter_max > 1
    figure()
    plot(1:steps, mean_iter)
    title('Solver Iterations')
    xlabel('Step k')
    ylabel('Iterations')
    xlim([1, steps]);
    ylim([0, optim.iter_max])
    legend(solvers)
end

%% Print performance measures to console
disp(table_perf)
disp(table_subtask)

%% Export to csv
if export
    % Write performance plot date into table, for exporting
    table_perf_plot = array2table([(1:steps)', mean_total],...
        'VariableNames', [{'step'}, solvers(:)']);
    
    writetable(table_perf_plot, [ name '_perf_plot.csv' ])
    writetable(table_perf,      [ name '_perf.csv' ],    'WriteRowNames', true)
    writetable(table_subtask,   [ name '_subtask.csv' ], 'WriteRowNames', true)
end
end
