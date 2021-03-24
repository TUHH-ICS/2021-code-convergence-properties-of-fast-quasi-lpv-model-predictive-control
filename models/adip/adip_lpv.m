%---------------------------------------------------------------------------------------------------
% For Paper
% "Convergence Properties of Fast quasi-LPV Model Predictive Control"
% by Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function [A, B] = adip_lpv(x, ~) %#codegen
%ADIP_LPV Calculates the LPV model matrices of the ADIP
%   This function calculates the model matrices of the LPV model of the ADIP, given the current
%   state of the system.
%
%   x   -> Current state of the system
%
%   A,B -> Discrete-time model matrices at the current state

%% Define constants
l_1  = 13.95e-2;  % Length of the arm
lg_1 =  6.975e-2; % Center of gravity of the arm
l_2  =  7.8e-2;   % Length of the pendulum
m_1  = 115e-3;    % Mass of the arm
m_h  = 130e-3;    % Mass of the encoder
m_2  =  73.1e-3;  % Mass of the pendulum
fv_1 = 1.3e-3;    % Coefficient of friction for the arm
fv_2 = 2.2e-5;    % Coefficient of frcition for the pendulum
g    = 9.81;      % Force of gravity

dT   = 0.01;      % Sampling time

%% Disect states
% For easier calculation in the following lines, we split the states into theta and its derivative

theta = x(1:2);
omega = x(3:4);

%% Build preliminary matrices
% These are the matrices used to define also the nonlinear model in the paper. The vector G is not
% defined here, as it is replaced by the matrix K

m21 = m_2*l_1*l_2;
M   = [ (m_1+m_2)*l_1^2             m21*cos(theta(1)-theta(2)) ;
        m21*cos(theta(1)-theta(2))  m_2*l_2^2                  ];
C   = [  fv_1                                 m21*sin(theta(1)-theta(2))*omega(2) ;
        -m21*sin(theta(1)-theta(2))*omega(1)  fv_2                                ];
K   = [ -(m_1*lg_1+(m_h+m_2)*l_1)*g*si(theta(1))   0                        ;
         0                                          -m_2*l_2*g*si(theta(2)) ];

% We cannot avoid inverting M, as it is needed for the input matrix B
Mi  = inv(M);

%% Assemble model matrices from parts
% From the previously defined parts, assemle the final model matrices A & B. At the same time, apply
% Euler's method for descritization.

A  = eye(4) + dT * [  zeros(2)   eye(2) ;
                     -Mi*K      -Mi*C    ];
B  = dT * [ zeros(2,1) ; Mi(:, 1) ];
end

function y = si(x)
%SI Defines the SI function
%   Matlab does not define the SI function, so we need to. Just taking sin(x)/x will result in a
%   singularity for x = 0, which is undesirable

    if x == 0
        y = 1;
    else
        y = sin(x) / x;
    end
end
