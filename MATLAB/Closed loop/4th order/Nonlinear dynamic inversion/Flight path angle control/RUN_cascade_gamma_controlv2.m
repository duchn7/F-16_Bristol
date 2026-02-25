function RUN_cascade_gamma_controlv2()
    % Initial state [alpha; V; q; theta]
    q0     = -0.0008;       % rad/s
    alpha0 = 0.0577;        % rad
    V0     = 266.2419;      % m/s
    theta0 = -0.0090;       % rad
    x0 = [alpha0; V0; q0; theta0];

    % Desired gamma (flight path angle)
    gamma_des = deg2rad(0);   

    % Simulation time
    tspan = [0 35];

    % Global variable to track previous control input
    global last_input 
    last_input = [-5;10000];  % Initial stabilator (deg) and thrust (N)

    % Call ODE solver
    [time_log, X] = ode45(@(t, x) wrapper(t, x, gamma_des), tspan, x0);

    % Reconstruct control input dstab at each output time
    dstab_log = zeros(2,length(time_log));
    for i = 1:length(time_log)
        [~, dstab_log(:,i)] = eom_NDIv2(time_log(i), X(i,:)', last_input, compute_theta_des(X(i,:)', gamma_des));
        last_input = dstab_log(:,i);
    end

    % ----------------- PLOTS ------------------
    figure,
    subplot(4,1,1); plot(time_log, X(:,1)*180/pi,'k','LineWidth',1); ylabel('\alpha (deg)');
    subplot(4,1,2); plot(time_log, X(:,2),'k','LineWidth',1); ylabel('V (m/s)');
    subplot(4,1,3); plot(time_log, X(:,3)*180/pi,'k','LineWidth',1); ylabel('q (deg/s)');
    subplot(4,1,4); plot(time_log, X(:,4)*180/pi,'k','LineWidth',1); ylabel('\theta (deg)'); xlabel('Time (s)');

    figure;
    subplot(2,1,1)
    gamma = X(:,4) - X(:,1);
    plot(time_log, gamma * 180/pi, 'b', 'LineWidth', 1.5); hold on;
    yline(gamma_des * 180/pi, 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('\gamma [deg]');
    legend('Actual \gamma', 'Desired \gamma');
    title('Flight Path Angle Tracking');

    subplot(2,1,2)
    plot(time_log, dstab_log(1,:), 'k', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Stabilator Deflection u [deg]');
    title('Control Input');

    % =============== Nested Function ==================
    function dxdt = wrapper(t, x, gamma_des)
        % Compute current gamma and gamma_dot
        alpha = x(1);
        q     = x(3);
        theta = x(4);

        % Estimate alphadot numerically
        eps = 1e-3;
        dx1 = eom(t, x, [last_input + eps; 25; 10000]);
        dx2 = eom(t, x, [last_input - eps; 25; 10000]);
        alphadot = (dx1(1) + dx2(1)) / 2;

        gamma = theta - alpha;
        gammadot = q - alphadot;

        % Outer-loop PD controller: gamma ? theta_des
        Kp = 2;
        Kd = 2;
        theta_des = theta + Kp * (gamma_des - gamma) - Kd * gammadot;

        % Inner-loop NDI controller
        [dxdt, ~] = eom_NDIv2(t, x, last_input, theta_des);
    end

    function theta_des = compute_theta_des(x, gamma_des)
        alpha = x(1);
        q     = x(3);
        theta = x(4);

        % Estimate alphadot for gamma_dot
        eps = 1e-3;
        dx1 = eom(0, x, [last_input + eps; 25; 10000]);
        dx2 = eom(0, x, [last_input - eps; 25; 10000]);
        alphadot = (dx1(1) + dx2(1)) / 2;

        gamma = theta - alpha;
        gammadot = q - alphadot;

        % Outer-loop PD controller again
        Kp = 2;
        Kd = 2;
        theta_des = theta + Kp * (gamma_des - gamma) - Kd * gammadot;
    end
end
