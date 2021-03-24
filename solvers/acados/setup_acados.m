function setup_acados()
%SETUP_ACADOS This function is used to set up the ACADOS toolbox.
%   This function will add the correct folders of the ACADOS installtion to
%   the Matlab path.
%   For Matlab to be able to find the ACADOS installation, the
%   environmental variable ACADOS_INSTALL_DIR has to be set.

% This function only need to be executed once, so set environmetal variable
% to save its state. This behaviour is copied from the ACADOS examples.
if ~strcmp(getenv('ENV_RUN'), 'true')
    
    %% Build correct paths to the ACADOS installation
    acados_dir           = fullfile(getenv('ACADOS_INSTALL_DIR'));
    casadi_dir           = fullfile(acados_dir, 'external', 'casadi-matlab');
    matlab_interface_dir = fullfile(acados_dir, 'interfaces', 'acados_matlab_octave');
    mex_template_dir     = fullfile(matlab_interface_dir, 'acados_template_mex');

    %% Add the required folders to the Matlab path
    addpath(matlab_interface_dir);
    addpath(mex_template_dir);
    addpath(casadi_dir);

    %% Signal the successful setup
    setenv('ENV_RUN', 'true');
end
end
