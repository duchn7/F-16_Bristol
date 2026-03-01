function RUN_INDIv2() % Code for theta pitch control, adapted to use ode45
    % Initial state [alpha; V; q; theta]
    q0     = -0.0008; % rad/s
    alpha0 = 0.0577; % rad
    V0     = 266.2419 ; % m/s
    theta0 = -0.0090; % rad
    x0 = [alpha0;V0;q0;theta0];

    % Desired alpha (radians)
    theta_des = deg2rad(5) +theta0;

    % Simulation time
    tspan = [0 20];

    % Global variable to track previous control input
    global last_dstab
    last_dstab = -5; % Initial stabilator deflection at t=0

    % Call ode45
    [time_log, X] = ode45(@(t, x) wrapper(t, x, theta_des), tspan, x0);

    % Reconstruct control input dstab at each output time
    % This is required as ODE45 doesn't store it so we re calculate it
    
    dstab_log = zeros(size(time_log));
    for i = 1:length(time_log)
        [~, dstab_log(i)] = eom_INDI(time_log(i), X(i,:)', last_dstab, theta_des);
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
    plot(time_log, X(:,4)*180/pi, 'b', 'LineWidth', 1.5); hold on;
    yline(theta_des*180/pi, 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('\theta [deg]');
    legend('Actual \theta', 'Desired \theta');
    title('Pitch Tracking');

    subplot(2,1,2)
    plot(time_log, dstab_log, 'k', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Stabilator Deflection u [deg]');
    title('Control Input');

    % =============== Nested Function ==================
    function dxdt = wrapper(t, x, theta_des)
        [dxdt, ~] = eom_INDI(t, x, last_dstab, theta_des); %embedded controller
    end
end
