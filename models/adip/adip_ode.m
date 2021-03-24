function dxdt = adip_ode(x, u)
%ADIP_ODE Implementation of the ODE for the ADIP
%   This function implements the ODE for the ADIP. This function can also be used to define the
%   symbolic expression required for both the acados & CasADi / Ipopt solvers.

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

%% Disect states
% For easier calculation in the following lines, we split the states into theta and its derivative
theta = x(1:2);
omega = x(3:4);

%% Equations of motion
m21 = m_2*l_1*l_2;
M   = [ (m_1+m_2)*l_1^2               m21*cos(theta(1)-theta(2)) ;
        m21*cos(theta(1) - theta(2))  m_2*l_2^2                  ];
C   = [  fv_1                                 m21*sin(theta(1)-theta(2))*omega(2) ;
        -m21*sin(theta(1)-theta(2))*omega(1)  fv_2                                ];
G   = [ -(m_1*lg_1+(m_h+m_2)*l_1)*g*sin(theta(1)) ;
        -m_2*l_2*g*sin(theta(2))                  ];

dxdt = [ omega                      ;
         M \ ([u; 0] - C*omega - G) ];
end
