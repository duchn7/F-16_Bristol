function [dxdt,dstab] = eom_INDI(t, x,dstab_current,theta_des)
% Function that combines equations of motion and incremental non-linear dynamic
% inversion---includes aero model function within this function
    % Unpack states
    alpha = x(1);
    V     = x(2);
    q     = x(3);
    theta = x(4);

    % Desired pitch command
   % theta_des = desired theta
    theta_des_dot = 0;
    theta_des_ddot = 0;

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
    T = 10000;     % fixed thrust
    
    
    % INDI will require either a 'measured' value from sensor or in our
    % case just using the equations of motion to see current theta double
    % dot
    qdot_measured = compute_qdot(x,dstab_current);
    
    % Compute aerodynamic coefficients including CM
    dstab_guess = dstab_current; %point of linearisation for derivative
    
    [CX, CZ, CM] = aero_model(alpha, V, q, dstab_guess, CG);  % function for aero model
    dCM_ddstab = deriv(alpha, V, q, dstab_guess, CG); % finds numerical derivative
    
    % Compute qdot = f(x) + g(x)*dstab
    common_factor = 0.5 * rho * V^2 * S * cbar / Iyy;
    %f_x = common_factor * (CM - dCM_ddstab * dstab_guess);  % Remove dstab's effect
    g_x = common_factor * dCM_ddstab;

    % Compute desired qdot
    qdot_des = -k1 * (theta - theta_des) - k2 * (q - theta_des_dot);

    % INDI law
    dstab = dstab_current + (1 / g_x) * (qdot_des - qdot_measured);

%     % Saturate dstab if needed
%     dstab = max(min(dstab, 25), -25);  % deg

% now calculate state derivatives
[CX, CZ, CM] = aero_model(alpha, V, q, dstab, CG);  % function for aero model

alphadot=q+(m*g*cos(theta-alpha)+0.5*rho*V^2*S*(CZ*cos(alpha)-CX*sin(alpha))-T*sin(alpha))/(m*V);
vdot=(-m*g*(sin(theta-alpha))+0.5*rho*V^2*S*(CX*cos(alpha)+CZ*sin(alpha))+T*cos(alpha))/m;
qdot=0.5*rho*V^2*S*cbar*CM/Iyy;
thetadot=q;

dxdt = [alphadot; vdot; qdot; thetadot];

end
