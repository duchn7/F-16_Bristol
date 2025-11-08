clear all; close all; 

%% Run simulation until the aircraft reaches an equilibrium
% Control input
de = -5;    % stabilator in deg (+- 25)

% Initial condition
alpha0 = 1.08578827850333; % rad
V0     = 85.9137949570426; % m/s
q0     = 0; % rad/s
theta0 = 0.184809486923954; % rad

u0 = de;
x0 = [alpha0; V0; q0; theta0];
options=simset('InitialState',x0,'RelTol',1e-3); % default RelTol: 1e-3

% Run simulation until the aircraft reaches an equilibrium
[t,x] = sim('f16_4_lin',500,options,[0 u0]);

figure
subplot(4,1,1); plot(t,x(:,1)*180/pi,'k','LineWidth',1); ylabel('\alpha (deg)'); title('Nonlinear simulation');
subplot(4,1,2); plot(t,x(:,2)       ,'k','LineWidth',1); ylabel('V (m/s)');
subplot(4,1,3); plot(t,x(:,3)*180/pi,'k','LineWidth',1); ylabel('q (deg/s)');
subplot(4,1,4); plot(t,x(:,4)*180/pi,'k','LineWidth',1); ylabel('\theta (deg)'); xlabel('t (s)');

%% Linearise with 1 input (stabilator)
[A, B, C, D] = linmod('f16_4_lin',x(end,:),u0);
sys=ss(A,B,C,D);

%% Step response (stabilator to pitch angle)
% figure; step(tf(sys(4,1))*180/pi) % simple method without changing stabiltor sign

step_ampl=-1; % deg
[x_lin,t_lin]=step(step_ampl*tf(sys(4,1)),600);

figure; 
plot(t_lin,x_lin*180/pi,'k','LineWidth',1);
xlabel('t (s)'); ylabel('\Delta\theta (deg)'); title('Linear simulation');

%% Frequency response (stabilator to pitch angle)
% figure; bode(tf(sys(4,1))*180/pi) % simple method without changing stabiltor sign

W=logspace(-3,+2,1000);
[MAG,PHASE]=bode(step_ampl*tf(sys(4,1))*180/pi,W);

figure; 
subplot(211); plot(W,20*log10(squeeze(MAG)),'k-','LineWidth',1); set(gca,'XScale', 'log'); xlim([0.001 100]); grid on;
ylabel('Magnitude (dB)');
subplot(212); plot(W,squeeze(PHASE),'k-','LineWidth',1); xlim([0.001 100]); grid on; set(gca,'XScale', 'log');
set(gca,'YTick',[-360:45:360]); xlabel('Frequency (rad/s)'); ylabel('Phase (deg)');
