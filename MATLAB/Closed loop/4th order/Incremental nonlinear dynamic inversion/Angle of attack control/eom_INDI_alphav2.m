function [dxdt,dstab] = eom_INDI_alphav2(t, x,dstab_current,alpha_des)
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
    k = 1; % 

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
    % case just using the equations of motion to see current alpha dot
    dx0 = eom(t, x, [dstab_current; CG; T]);
    alphadot_measured = dx0(1);
    
    % Numerical derivative to compute g(x)
    [dCX_ddstab,dCZ_ddstab,~] = deriv_alpha(alpha, V, q, dstab_current, CG);
    
    % Compute g(x)
    g_x = (0.5 * rho * V * S / m) * (dCZ_ddstab * cos(alpha) - dCX_ddstab * sin(alpha));
    
%     eps = 0.1; % Small change in dstab
% 
%     % alphadot at dstab + eps
%     dx_plus = eom(t, x, [dstab_current + eps; CG; T]);
%     alphadot_plus = dx_plus(1);
% 
%     % alphadot at dstab - eps
%     dx_minus = eom(t, x, [dstab_current - eps; CG; T]);
%     alphadot_minus = dx_minus(1);
% 
%     % Numerical derivative
%     g_x = (alphadot_plus - alphadot_minus) / (2 * eps);

    % Compute desired qdot
    alphadot_des = -k * (alpha - alpha_des) ;
    
    % INDI law
    dstab = dstab_current + (1 / g_x) * (alphadot_des - alphadot_measured);

%     % Saturate dstab if needed
    dstab = max(min(dstab, 25), -25);  % deg

% now calculate state derivatives

[CX, CZ, CM] = aero_model(alpha, V, q, dstab, CG);  % function for aero model

alphadot=q+(m*g*cos(theta-alpha)+0.5*rho*V^2*S*(CZ*cos(alpha)-CX*sin(alpha))-T*sin(alpha))/(m*V);
vdot=(-m*g*(sin(theta-alpha))+0.5*rho*V^2*S*(CX*cos(alpha)+CZ*sin(alpha))+T*cos(alpha))/m;
qdot=0.5*rho*V^2*S*cbar*CM/Iyy;
thetadot=q;

dxdt = [alphadot; vdot; qdot; thetadot];

end
