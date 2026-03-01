function [dxdt, control_out] = eom_NDI_2input(t, x, control_in, V_des, theta_des)

% Unpack states
alpha = x(1);
V     = x(2);
q     = x(3);
theta = x(4);
% Unpack controller
dstab_current = control_in(1);
thrust_current = control_in(2);
    
% theta_des = desired theta
theta_des_dot = 0;
    

% PD gains for theta
k1 = 1; k2 = 1; % 
% Gain for V
k3 = 1;

% Aero and aircraft constants
cbar = 3.4503;
S = 27.87;
rho = 0.458312441644953;
g    = 9.81             ; % m/s^2
C7 = 1.322E-5;
Iyy = 1/C7;
RM   = 1.076E-4; % 1/m
m=1/RM;

% Assume CG constant
CG = 25;

% Compute aerodynamic coefficients 
dstab_guess = dstab_current; %point of linearisation for derivative
    
[CX, CZ, CM] = aero_model(alpha, V, q, dstab_guess, CG);  % function for aero model
[dCX_ddstab,dCZ_ddstab,dCM_ddstab] = ...
        deriv_alpha(alpha, V, q, dstab_guess, CG); % finds numerical derivative
    
CZ_sep = CZ - dCZ_ddstab* dstab_guess; %separated into independant of dstab input
CX_sep = CX - dCX_ddstab* dstab_guess;
CM_sep = CM - dCM_ddstab* dstab_guess;

% Compute qdot = f1(x) + g1(x)*[dstab;thrust] for theta control
common_factor = 0.5 * rho * V^2 * S * cbar / Iyy;
f_x1 = common_factor * (CM_sep);        % Natural dynamics
g_x11 = common_factor * dCM_ddstab;     % dstab effect on qdot
g_x12 = 0 ;                             % Thrust has no influence on qdot

% Compute desired qdot
qdot_des = -k1 * (theta - theta_des) - k2 * (q - theta_des_dot);

% Compute vdot = f2(x)+g2(x)*[dstab;thrust]
f_x2  = (-m*g*sin(theta - alpha) + 0.5*rho*V^2*S*(CX_sep*cos(alpha) + CZ_sep*sin(alpha)) )/m;
g_x21 = (0.5*rho*V^2*S*(dCX_ddstab*cos(alpha) + dCZ_ddstab*sin(alpha)) )/m;
g_x22 = (cos(alpha))/m;

% Compute desired Vdot
Vdot_des = -k3 * (V - V_des) ;

f = [f_x1;f_x2];
G = [g_x11,g_x12;g_x21,g_x22];
desired = [qdot_des ; Vdot_des] ;
control_out = G \ (desired - f) ;

dstab = control_out(1);
thrust = control_out(2);
% Saturate thrust
thrust = max(min(thrust, 20000), 0);  % thrust
control_out(2) = thrust;
% now calculate state derivatives
[CX, CZ, CM] = aero_model(alpha, V, q, dstab, CG);  % function for aero model

alphadot=q+(m*g*cos(theta-alpha)+0.5*rho*V^2*S*(CZ*cos(alpha)-CX*sin(alpha))-thrust*sin(alpha))/(m*V);
vdot=(-m*g*(sin(theta-alpha))+0.5*rho*V^2*S*(CX*cos(alpha)+CZ*sin(alpha))+thrust*cos(alpha))/m;
qdot=0.5*rho*V^2*S*cbar*CM/Iyy;
thetadot=q;

dxdt = [alphadot; vdot; qdot; thetadot];

end
