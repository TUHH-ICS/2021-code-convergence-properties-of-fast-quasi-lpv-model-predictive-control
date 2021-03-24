function dxdt = dynamic_unicycle_ode(x, u)
%DYNAMIC_UNICYCLE_ODE Implementation of the ODE for the dynamic unicycle
%   This function implements the ODE for the dynamic unicycle. This function can also be used to
%   define the symbolic expression required for both the acados & CasADi / Ipopt solvers.

dxdt = [ x(3) * cos(x(4)) ;
         x(3) * sin(x(4)) ;
         u(1)             ;
         x(5)             ;
         u(2)             ];
end
