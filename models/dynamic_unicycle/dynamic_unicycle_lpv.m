function [A, B] = dynamic_unicycle_lpv(x, ~) %#codegen
%DYNAMIC_UNICYCLE_LPV Calculates the LPV model matrices of the dynamic unicycle
%   This function calculates the model matrices of the LPV model of the dynamic unicycle, given the
%   current state of the system.
%
%   x   -> Current state of the system
%
%   A,B -> Discrete-time model matrices at the current state

dT = 0.1;

A  = dT * [ 0 0 cos(x(4)) 0 0 ;
            0 0 sin(x(4)) 0 0 ;
            0 0 0         0 0 ;
            0 0 0         0 1 ;
            0 0 0         0 0 ] + eye(5);
B  = dT * [ 0 0 ;
            0 0 ;
            1 0 ;
            0 0 ;
            0 1 ];
end
