function [dxdt,control_out] = eom_NDIv2(t, x,control_in,theta_des)
% Function that combines equations of motion and non-linear dynamic
% inversion---includes aero model function within this function
    % Unpack states
    alpha = x(1);
    V     = x(2);
    q     = x(3);
    theta = x(4);
    
    dstab_current = control_in(1);
    thrust_current = control_in(2);
    % Desired pitch command
   % theta_des = desired theta
    theta_des_dot = 0;
    

    % PD gains
    k1 = 1; k2 = 1; % 

    % Aero and aircraft constants
    cbar = 3.4503;
    S = 27.87;
    rho = 0.458312441644953;
    g    = 9.81             ; % m/s^2
    C7 = 1.322E-5;
    Iyy = 1/C7;
    RM   = 1.076E-4; % 1/m
    m=1/RM;
    
    % Assume CG and thrust are fixed:
    CG = 25;      % fixed CG
    
    % Compute aerodynamic coefficients including CM
    dstab_guess = dstab_current; %point of linearisation for derivative
    
    [~, ~, CM] = aero_model(alpha, V, q, dstab_guess, CG);  % function for aero model
    dCM_ddstab = deriv(alpha, V, q, dstab_guess, CG); % finds numerical derivative
    
    % Compute qdot = f(x) + g(x)*dstab
    common_factor = 0.5 * rho * V^2 * S * cbar / Iyy;
    f_x = common_factor * (CM - dCM_ddstab * dstab_guess);  % Remove dstab's effect
    g_x = common_factor * dCM_ddstab;

    % Compute desired qdot
    qdot_des = -k1 * (theta - theta_des) - k2 * (q - theta_des_dot);

    % Nonlinear dynamic inversion: solve for dstab
    dstab = (qdot_des - f_x) / g_x;

    % Saturate dstab, look up table bounds at -25 and 25
    dstab = max(min(dstab, 25), -25);  % deg
    
    % Fixed thrust
    thrust = thrust_current;
    control_out = [dstab;thrust];
% now calculate state derivatives
[CX, CZ, CM] = aero_model(alpha, V, q, dstab, CG);  % function for aero model

alphadot=q+(m*g*cos(theta-alpha)+0.5*rho*V^2*S*(CZ*cos(alpha)-CX*sin(alpha))-thrust*sin(alpha))/(m*V);
vdot=(-m*g*(sin(theta-alpha))+0.5*rho*V^2*S*(CX*cos(alpha)+CZ*sin(alpha))+thrust*cos(alpha))/m;
qdot=0.5*rho*V^2*S*cbar*CM/Iyy;
thetadot=q;

dxdt = [alphadot; vdot; qdot; thetadot];

end
