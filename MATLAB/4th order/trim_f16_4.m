%% Trimming for constant flight path angle gamma = theta - alpha
clear all; close all; clc
global CG gamma Vtrim x_cost

%% Set up
% Specify the trim condition
CG     = 25; % CG position (%MAC)
gamma  = 0; % flight path angle (deg) - 0 for level flight
Vtrim  = 200; % velocity (m/s)

% Guess the trim solution
alpha_guess = 5; % angle of attack (deg)
de_guess    = -5; % stabilator (deg)
T_guess     = 10000; % thrust (N)

% Simulation times
t_sim1 = 300; % initial guess (sec)
t_sim2 = 300; % trim solution (sec)

%% Simulate using guess parameters
alpha0 = alpha_guess/180*pi; theta0 = (alpha_guess+gamma)/180*pi;
x0_1 = [alpha0; Vtrim; 0; theta0];
u0_1 = [de_guess; CG; T_guess];
[t1,x1] = ode45(@eom_4,[0 t_sim1],x0_1,'',u0_1); % MATLAB
% [t1,x1] = ode45(@eom_4_mex,[0 t_sim1],x0_1,'',u0_1); % Accelerated

figure
subplot(5,2,1); plot(t1 ,x1(:,1)         *180/pi,'k','LineWidth',1); ylabel('\alpha (deg)'); title('Initial guess');
subplot(5,2,3); plot(t1 ,x1(:,2)                ,'k','LineWidth',1); ylabel('V (m/s)');
subplot(5,2,5); plot(t1 ,x1(:,3)         *180/pi,'k','LineWidth',1); ylabel('q (deg/s)');
subplot(5,2,7); plot(t1 ,x1(:,4)         *180/pi,'k','LineWidth',1); ylabel('\theta (deg)');
subplot(5,2,9); plot(t1,(x1(:,4)-x1(:,1))*180/pi,'k','LineWidth',1); ylabel('\gamma (deg)'); xlabel('t (s)');

%% Find the trim solution
[trimvar,fval]=fminsearch(@cost,[de_guess, T_guess, alpha0]);

% Print trim solution 
fprintf(['TRIM CONDITION:\n'])
disp(['   gamma      = ',num2str(gamma),' deg'])
disp(['   CG         = ',num2str(CG),'% MAC'])
disp(['   velocity   = ',num2str(x_cost(2)),' m/s'])
fprintf(['TRIM SOLUTION:\n'])
disp(['   alpha      = ',num2str(x_cost(1)*180/pi),' deg'])
disp(['   theta      = ',num2str(x_cost(4)*180/pi),' deg'])
disp(['   stabilator = ',num2str(trimvar(1)),' deg'])
disp(['   thrust     = ',num2str(trimvar(2)),' N'])

%% Verify the trim solution
x0_2 = [trimvar(3); Vtrim; 0; (trimvar(3)+gamma/180*pi)];
u0_2 = [trimvar(1); CG; trimvar(2)];

[t2,x2] = ode45(@eom_4,[0 t_sim2],x0_2,'',u0_2); % MATLAB
% [t2,x2] = ode45(@eom_4_mex,[0 t_sim2],x0_2,'',u0_2); % Accelerated

subplot(5,2,2);  plot(t2 ,x2(:,1)         *180/pi,'k','LineWidth',1); ylabel('\alpha (deg)'); title('Trim solution');
subplot(5,2,4);  plot(t2 ,x2(:,2)                ,'k','LineWidth',1); ylabel('V (m/s)');
subplot(5,2,6);  plot(t2 ,x2(:,3)         *180/pi,'k','LineWidth',1); ylabel('q (deg/s)');
subplot(5,2,8);  plot(t2 ,x2(:,4)         *180/pi,'k','LineWidth',1); ylabel('\theta (deg)');
subplot(5,2,10); plot(t2,(x2(:,4)-x2(:,1))*180/pi,'k','LineWidth',1); ylabel('\gamma (deg)'); xlabel('t (s)');

%% Cost function
function J = cost(trimvar0)
    global CG gamma Vtrim x_cost

    u_cost  = [trimvar0(1); CG ; trimvar0(2)]; 
    x_cost  = [trimvar0(3); Vtrim; 0; (trimvar0(3)+gamma/180*pi)];          

    dx = eom_4(0,x_cost,u_cost); % MATLAB
%     dx = eom_4_mex(0,x_cost,u_cost); % Accelerated

    J = dx(1)^2 + 0.01*dx(2)^2 + dx(3)^2 + dx(4)^2 ; % alphadot^2 + 0.01*Vdot^2 + qdot^2 + thetadot^2;
end