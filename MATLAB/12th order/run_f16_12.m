clear all; close all;

%% Set up
% Select a pair of control input (CI) and initial condition (IC)
% CI 1, IC 1: fully-developed spin
% CI 2, IC 2: deep stall
% CI 1, IC 2: spin entry from deep stall
% CI 2, IC 1: spin self-recovery (back to deep stall)

% CI 1
da = -20; % aileron in deg (+-21.5)
de = -5; % stabilator deg (+-25)
dr = 0; % rudder in deg (+-30) 
CG = 37.5; % CG position in %MAC (25.0 to 37.5)
T  = 10000; % thrust in N

% IC 1
alpha0 = 1.163325534316547; % rad
beta0  = -0.0142233141487139; % rad
V0     = 92.2774155855499; % m/s
p0     = -0.249806083516482; % rad/s
q0     = 0.239517453942822; % rad/s
r0     = -0.582082779189844; % rad/s
phi0   = -0.113403721456400; % rad
theta0 = -0.414023981066343; % rad
psi0   = 0; % rad
X0     = 0; % m
Y0     = 0; % m
Z0     = 0; % m

% % CI 2
% da = 0; % aileron in deg (+-21.5)
% de = 0; % stabilator deg (+-25)
% dr = 0; % rudder in deg (+-30) 
% CG = 37.5; % CG position in %MAC (25.0 to 37.5)
% T  = 10000; % thrust in N

% % IC 2
% alpha0 = 1.08578827850333; % rad
% beta0  = 0; % rad
% V0     = 85.9137949570426; % m/s
% p0     = 0; % rad/s
% q0     = 0; % rad/s
% r0     = 0; % rad/s
% phi0   = 0; % rad
% theta0 = 0.184809486923954; % rad
% psi0   = 0; % rad
% X0     = 0; % m
% Y0     = 0; % m
% Z0     = 0; % m

%% Simulate and plot
x0 = [alpha0; beta0; V0; p0; q0; r0; phi0; theta0; psi0; X0; Y0; Z0];
u0 = [da; de; dr; CG; T];
options = odeset('RelTol',1e-5); % default: 1e-3
tic; [t,x] = ode45(@eom_12,[0 10],x0,options,u0); toc % MATLAB
% tic; [t,x] = ode45(@eom_12_mex,[0 10],x0,options,u0); toc % Accelerated

figure
subplot(3,4,1);  plot(t,x(:,1)*180/pi,'k','LineWidth',1); ylabel('\alpha (deg)');
subplot(3,4,5);  plot(t,x(:,2)*180/pi,'k','LineWidth',1); ylabel('\beta (deg)');
subplot(3,4,9);  plot(t,x(:,3)       ,'k','LineWidth',1); ylabel('V (m/s)'); xlabel('t (s)');
subplot(3,4,2);  plot(t,x(:,4)*180/pi,'k','LineWidth',1); ylabel('p (deg/s)'); 
subplot(3,4,6);  plot(t,x(:,5)*180/pi,'k','LineWidth',1); ylabel('q (deg/s)');
subplot(3,4,10); plot(t,x(:,6)*180/pi,'k','LineWidth',1); ylabel('r (deg/s)');  xlabel('t (s)');
subplot(3,4,3);  plot(t,x(:,7)*180/pi,'k','LineWidth',1); ylabel('\phi (deg)');
subplot(3,4,7);  plot(t,x(:,8)*180/pi,'k','LineWidth',1); ylabel('\theta (deg)');
subplot(3,4,11); plot(t,x(:,9)*180/pi,'k','LineWidth',1); ylabel('\psi (deg)'); xlabel('t (s)');
subplot(3,4,4);  plot(t,x(:,10)      ,'k','LineWidth',1); ylabel('X (m)');
subplot(3,4,8);  plot(t,x(:,11)      ,'k','LineWidth',1); ylabel('Y (m)');
subplot(3,4,12); plot(t,x(:,12)      ,'k','LineWidth',1); ylabel('Z (m)'); xlabel('t (s)'); set(gca, 'YDir', 'reverse')

%% Plot trajectory
scale      = 1; 
tplot_step = 0.05; % step time for the interpolated data (sec)
tplot_disp = 1;    % plot an airframe once every __ second of simulated time

t_3d = (0:tplot_step:t(end))';
x_3d = interp1(t,x,t_3d);
gap  = tplot_disp/tplot_step;

figure
trajectory3z( x_3d(:,10), x_3d(:,11), x_3d(:,12), ...
             -x_3d(:,8) , x_3d(:,7) , x_3d(:,9) , ...
             0.1/scale, gap);
set(gca,'Ydir','reverse','Zdir','reverse')