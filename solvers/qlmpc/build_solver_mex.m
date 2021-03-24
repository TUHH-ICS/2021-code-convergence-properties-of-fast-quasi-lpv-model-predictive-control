%---------------------------------------------------------------------------------------------------
% For Paper
% "Convergence Properties of Fast quasi-LPV Model Predictive Control"
% by Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function build_solver_mex(model, optim)
%BUILD_SOLVER_MEX Take the qLMPC Matlab solvers and compile them into native MEX-functions to
%improve the performance of the solvers
%   This function searches for the Matlab-based qLMPC solvers located in the mex_stubs folder and
%   compiles them into native MEX-functions. If the optional 2nd optim parameter is given, they are 
%   specialized to one specific horizon length N and cost function parameters Q & R.
%
%   model -> Model for which the solvers should be build
%   optim -> Struct with optimization parameters [Optional]

solver_name = sprintf('%s_solver', model.name);
solver_vel_name = sprintf('%s_solver_vel', model.name);

%% Prepare function arguments
% If the 2nd parameter is given, we can specialize the solver for the parameters given in that
% struct. Otherwise, we keep them as general parameters.
if nargin >= 2
    arg_optim = coder.Constant(optim);
    arg_x     = coder.newtype('double', [double(optim.N+1) * model.nx, 1]);
    arg_u     = coder.newtype('double', [double(optim.N) * model.nu, 1]);
else
    type_optim   = struct;
    type_optim.N = coder.newtype('uint32');
    type_optim.Q = coder.newtype('double', [model.nx, model.nx]);
    type_optim.R = coder.newtype('double', [model.nu, model.nu]);
    type_optim.iter_max = coder.newtype('uint32');
    type_optim.rel_tol  = coder.newtype('double');
    
    arg_optim = coder.newtype('struct', type_optim);
    arg_x     = coder.newtype('double', [inf, 1], [1, 0]);
    arg_u     = coder.newtype('double', [inf, 1], [1, 0]);
end

arg_x0 = coder.newtype('double', [model.nx, 1]);
arg    = {arg_optim, arg_x0, arg_x, arg_u};

%% Create build folder if required
if ~exist('./build', 'dir')
    mkdir('build')
end
addpath('build')

%% Build optimized MEX functions
disp('Building MEX functions for qLMPC solvers...')
codegen(solver_name, '-o', 'build/ocp_qlmpc', '-args', arg, '-nargout', 3)
codegen(solver_vel_name, '-o', 'build/ocp_qlmpc_vel', '-args', arg, '-nargout', 3)
disp('Finished building MEX functions!')
end
