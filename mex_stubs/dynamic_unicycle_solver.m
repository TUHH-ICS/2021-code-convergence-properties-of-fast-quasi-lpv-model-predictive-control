function [u_opt, x_opt, stats] = dynamic_unicycle_solver(optim, x0, x, u) %#codegen
%DYNAMIC_UNICYCLE_SOLVER Stub that specializes the qLMPC solver for the dynamic unicycle model
%   This stub function is required because the Matlab coder does not allow function handles as input
%   arguments, even if they are given as constants. Additionally, this allows us to remove some
%   superfluous parameters at the same time, such as nx & nu.
%
%   optim -> Struct with optimization parameters
%   x0    -> Value of the state at the initial time
%   x     -> State trajectory used for warmstarting the solver
%   u     -> Input trajectory used for warmstarting the solver
%
%   u_opt -> Optimal control trajectory
%   x_opt -> Optimal state trajectory
%   stats -> Struct with timings and iteration information

[u_opt, x_opt, stats] = qlmpc_solver(optim, @dynamic_unicycle_lpv, 5, 2, x0, x, u);
end
