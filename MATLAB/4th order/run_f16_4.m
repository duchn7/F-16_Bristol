clear all; close all; 

% Control inputs
de = -5;    % stabilator in deg (+- 25)
CG = 25;    % centre of gravity position in %MAC (25.0 to 37.5)
T  = 10000; % thrust in N

% Initial condition
alpha0 = 1.08578827850333; % rad
V0     = 85.9137949570426; % m/s
q0     = 0; % rad/s
theta0 = 0.184809486923954; % rad

u0 = [de; CG; T];
x0 = [alpha0; V0; q0; theta0];
options = odeset('RelTol',1e-4); % default: 1e-3
tic; [t,x] = ode45(@eom_4,[0 500],x0,options,u0); toc % MATLAB
% tic; [t,x] = ode45(@eom_4_mex,[0 500],x0,options,u0); toc % Accelerated

figure
subplot(4,1,1); plot(t,x(:,1)*180/pi,'k','LineWidth',1); ylabel('\alpha (deg)');
subplot(4,1,2); plot(t,x(:,2)       ,'k','LineWidth',1); ylabel('V (m/s)');
subplot(4,1,3); plot(t,x(:,3)*180/pi,'k','LineWidth',1); ylabel('q (deg/s)');
subplot(4,1,4); plot(t,x(:,4)*180/pi,'k','LineWidth',1); ylabel('\theta (deg)'); xlabel('t (s)');