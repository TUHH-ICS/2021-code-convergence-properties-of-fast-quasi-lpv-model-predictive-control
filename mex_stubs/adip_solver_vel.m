function [u_opt, x_opt, stats] = adip_solver_vel(optim, x0, x, u) %#codegen
%ADIP_SOLVER_VEL Stub that specializes the exact qLMPC solver for the ADIP model
%   This stub function is required because the Matlab coder does not allow function handles as input
%   arguments, even if they are given as constants. Additionally, this allows us to remove some
%   superfluous parameters at the same time, such as nx, nu & nd.
%
%   optim -> Struct with optimization parameters
%   x0    -> Value of the state at the initial time
%   x     -> State trajectory used for warmstarting the solver
%   u     -> Input trajectory used for warmstarting the solver
%
%   u_opt -> Optimal control trajectory
%   x_opt -> Optimal state trajectory
%   stats -> Struct with timings and iteration information

[u_opt, x_opt, stats] = qlmpc_vel_solver(optim, @adip_vel, 4, 1, 4, x0, x, u);
end
