%---------------------------------------------------------------------------------------------------
% For Paper
% "Convergence Properties of Fast quasi-LPV Model Predictive Control"
% by Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function [A, B, Bd, d] = dynamic_unicycle_vel(x, ~) %#codegen
%DYNAMIC_UNICYCLE_VEL Calculates the LPV model matrices of the fictitious dynamic unicycle model
%   This function calculates the model matrices of the LPV model of the dynamic unicycle, given the
%   current state of the system. Here, not the standard LPV model is implemented, but the fictitious
%   one needed for the exact variant of qLMPC.
%
%   x      -> Current state of the system
%
%   A,B,Bd -> Discrete-time model matrices at the current state
%   d      -> Disturbance vector at the current state

dT = 0.1;

B  = dT * [ 0 0 ;
            0 0 ;
            1 0 ;
            0 0 ;
            0 1 ];
Bd = dT * [  x(3)*sin(x(4)) ;
            -x(3)*cos(x(4)) ;
             0              ;
             0              ;
             0              ];  
A  = eye(5) + dT * [ 0 0 cos(x(4)) -x(3)*sin(x(4)) 0 ;
                     0 0 sin(x(4))  x(3)*cos(x(4)) 0 ;
                     0 0 0          0              0 ;
                     0 0 0          0              1 ;
                     0 0 0          0              0 ];
d = x(4);

end
