function RUN_alpha_v6() % Code for alpha AoA control, adapted to use ode45 and INDI

    % Initial state [alpha; V; q; theta]
    q0     = -0.0008; % rad/s
    alpha0 = 0.0577; % rad
    V0     = 266.2419 ; % m/s
    theta0 = -0.0090; % rad
    x0 = [alpha0;V0;q0;theta0];

    % Desired alpha (radians)
    alpha_des = deg2rad(6);

    % Simulation time
    tspan = [0 7];

    % Global variable to track previous control input
    global last_dstab
    last_dstab = -5;  % Initial stabilator deflection at t=0
%     xxx =wrapper(0,x0,alpha_des);
    options = odeset('RelTol',1e-3);
    % Call ode45
    [time_log, X] = ode45(@(t, x) wrapper(t, x, alpha_des), tspan, x0, options);

    % Reconstruct control input dstab at each output time
    % This is required as ODE45 doesn't store it so we re calculate it
    
    dstab_log = zeros(size(time_log));
    for i = 1:length(time_log)
        [~, dstab_log(i)] = eom_INDI_alphav2(time_log(i), X(i,:)', last_dstab, alpha_des);
        last_dstab = dstab_log(i);  % Update global value
    end

    % ----------------- PLOTS ------------------
    figure,
    subplot(4,1,1); plot(time_log, X(:,1)*180/pi,'k','LineWidth',1); ylabel('\alpha (deg)');
    subplot(4,1,2); plot(time_log, X(:,2),'k','LineWidth',1); ylabel('V (m/s)');
    subplot(4,1,3); plot(time_log, X(:,3)*180/pi,'k','LineWidth',1); ylabel('q (deg/s)');
    subplot(4,1,4); plot(time_log, X(:,4)*180/pi,'k','LineWidth',1); ylabel('\theta (deg)'); xlabel('Time (s)');

    figure;
    subplot(2,1,1)
    plot(time_log, X(:,1)*180/pi, 'b', 'LineWidth', 1.5); hold on;
    yline(alpha_des*180/pi, 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('\alpha [deg]');
    legend('Actual \alpha', 'Desired \alpha');
    title('AoA Tracking');

    subplot(2,1,2)
    plot(time_log, dstab_log, 'k', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Stabilator Deflection u [deg]');
    title('Control Input');

    % =============== Nested Function ==================
    function dxdt = wrapper(t, x, alpha_des)
        [dxdt, ~] = eom_INDI_alphav2(t, x, last_dstab, alpha_des); % embedded controller
    end
end
