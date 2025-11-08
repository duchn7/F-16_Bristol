%% Trim for coordinated level turn (gamma = 0 and beta = 0)
clear all; close all; clc
global CG n Vtrim x_cost

%% Set up
% Specify the trim condition
CG     = 25; % CG position (%MAC)
n      = 2; % load factor
Vtrim  = 200; % velocity (m/s)

% Guess the trim solution
aoa_guess = 5;     % angle of attack (deg)
phi_guess = 5;     % roll angle (deg)
da_guess  = -0.5;  % aileron (deg)
de_guess  = -5;    % stabilator (deg)
dr_guess  = -0.5;  % rudder (deg)
T_guess   = 10000; % thrust (N)

% Simulation times
t_sim1 = 100; % initial guess (sec)
t_sim2 = 100; % trim solution (sec)

%% Simulate using guess parameters
alpha0 = aoa_guess/180*pi; 
beta0  = 0; 
p0     = 5/180*pi; 
q0     = 5/180*pi; 
r0     = 5/180*pi; 
phi0   = phi_guess/180*pi; 
theta0 = 5/180*pi;

x0_1 = [alpha0; beta0; Vtrim; p0; q0; r0; phi0; theta0];
u0_1 = [da_guess; de_guess; dr_guess; CG; T_guess];
[t1,x1] = ode45(@eom_8,[0 t_sim1],x0_1,odeset('MaxStep',0.2),u0_1); % MATLAB
% [t1,x1] = ode45(@eom_8_mex,[0 t_sim1],x0_1,odeset('MaxStep',0.2),u0_1); % Accelerated

figure
subplot(3,3,1); plot(t1,x1(:,1)*180/pi,'k','LineWidth',1); ylabel('\alpha (deg)');
subplot(3,3,4); plot(t1,x1(:,2)*180/pi,'k','LineWidth',1); ylabel('\beta (deg)');
subplot(3,3,7); plot(t1,x1(:,3)       ,'k','LineWidth',1); ylabel('V (m/s)'); xlabel('t (s)');
subplot(3,3,2); plot(t1,x1(:,4)*180/pi,'k','LineWidth',1); ylabel('p (deg/s)'); title('Initial guess');
subplot(3,3,5); plot(t1,x1(:,5)*180/pi,'k','LineWidth',1); ylabel('q (deg/s)');
subplot(3,3,8); plot(t1,x1(:,6)*180/pi,'k','LineWidth',1); ylabel('r (deg/s)');  xlabel('t (s)');
subplot(3,3,3); plot(t1,x1(:,7)*180/pi,'k','LineWidth',1); ylabel('\phi (deg)');
subplot(3,3,6); plot(t1,x1(:,8)*180/pi,'k','LineWidth',1); ylabel('\theta (deg)'); xlabel('t (s)');

%% Find the trim solution
options = optimset('TolFun',1.0e-08,'TolX',1.0e-08, 'MaxIter',6000,'MaxFunEvals',6000);
[trimvar,fval]=fminsearch(@cost,[alpha0, phi0, da_guess, de_guess, dr_guess, T_guess],options);

% Print trim solution 
fprintf(['TRIM CONDITION:\n'])
disp(['   CG       = ',num2str(CG),' %MAC'])
disp(['   n        = ',num2str(n),' g'])
disp(['   velocity = ',num2str(x_cost(3)),' m/s'])
fprintf(['TRIM SOLUTION:\n'])
disp(['   alpha      = ',num2str(trimvar(1)*180/pi),' deg'])
disp(['   theta      = ',num2str(x_cost(8)*180/pi),' deg'])
disp(['   phi        = ',num2str(trimvar(2)*180/pi),' deg'])
disp(['   aileron    = ',num2str(trimvar(3)),' deg'])
disp(['   stabilator = ',num2str(trimvar(4)),' deg'])
disp(['   rudder     = ',num2str(trimvar(5)),' deg'])
disp(['   thrust     = ',num2str(trimvar(6)),' N'])

%% Verify the trim solution
x0_2 = [trimvar(1); 0; Vtrim; x_cost(4); x_cost(5); x_cost(6); trimvar(2); x_cost(8)];
u0_2 = [trimvar(3); trimvar(4); trimvar(5); CG; trimvar(6)];

[t2,x2] = ode45(@eom_8,[0 t_sim2],x0_2,odeset('MaxStep',0.2),u0_2); % MATLAB
% [t2,x2] = ode45(@eom_8_mex,[0 t_sim2],x0_2,odeset('MaxStep',0.2),u0_2); % Accelerated

figure
subplot(3,3,1); plot(t2,x2(:,1)*180/pi,'k','LineWidth',1); ylabel('\alpha (deg)');
subplot(3,3,4); plot(t2,x2(:,2)*180/pi,'k','LineWidth',1); ylabel('\beta (deg)');
subplot(3,3,7); plot(t2,x2(:,3)       ,'k','LineWidth',1); ylabel('V (m/s)'); xlabel('t (s)');
subplot(3,3,2); plot(t2,x2(:,4)*180/pi,'k','LineWidth',1); ylabel('p (deg/s)'); title('Trim solution');
subplot(3,3,5); plot(t2,x2(:,5)*180/pi,'k','LineWidth',1); ylabel('q (deg/s)');
subplot(3,3,8); plot(t2,x2(:,6)*180/pi,'k','LineWidth',1); ylabel('r (deg/s)');  xlabel('t (s)');
subplot(3,3,3); plot(t2,x2(:,7)*180/pi,'k','LineWidth',1); ylabel('\phi (deg)');
subplot(3,3,6); plot(t2,x2(:,8)*180/pi,'k','LineWidth',1); ylabel('\theta (deg)'); xlabel('t (s)');

%% Cost function
function J = cost(trimvar0)
    global CG n Vtrim x_cost
    
    u_cost     = zeros(5,1);
    u_cost(1)  = trimvar0(3); % aileron (deg)
    u_cost(2)  = trimvar0(4); % stabilator (deg)
    u_cost(3)  = trimvar0(5); % rudder (deg)
    u_cost(4)  = CG;          % CG (%MAC)
    u_cost(5)  = trimvar0(6); % thrust (N)

    phi_w = atan(sqrt(n^2-1));
    g = 9.81;
    
    x_cost     = zeros(8,1);
    x_cost(1)  =  trimvar0(1);                                  % alpha (rad)
    x_cost(2)  =  0;                                            % beta  (rad)
    x_cost(3)  =  Vtrim;                                        % V     (m/s)
    x_cost(4)  = -g/x_cost(3) * sin(phi_w)   * sin(x_cost(1));  % p     (rad/s)
    x_cost(5)  =  g/x_cost(3) * sin(phi_w)^2 / cos(phi_w);      % q     (rad/s)
    x_cost(6)  =  g/x_cost(3) * sin(phi_w)   * cos(x_cost(1));  % r     (rad/s)
    x_cost(7)  =  trimvar0(2);                                  % phi   (rad)
    x_cost(8)  =  atan(tan(x_cost(1))*cos(x_cost(7)));          % theta (rad)        

    dx = eom_8(0,x_cost,u_cost); % MATLAB
%     dx = eom_8_mex(0,x_cost,u_cost); % Accelerated

    J = dx(1)^2 + dx(2)^2 + 0.01*dx(3)^2 + dx(4)^2 + dx(5)^2 + dx(6)^2 + dx(7)^2 + dx(8)^2;
end