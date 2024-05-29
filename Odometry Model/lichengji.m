% Initialize parameters
x = 0; y = 0; theta = 0; % Initial position and orientation
r = 1; % Wheel radius
L_2 = 2; % Distance between the two rear wheels
T_s = 0.1; % Sampling period
t_end = 100; % Total time
phi_dot = 0.1; % Constant angular velocity for simplicity

% Standard deviations for Gaussian noise
std_dev_phi = 1; % Standard deviation for angular velocity of the wheels

% Initialize positions
x_est = x; y_est = y; theta_est = theta;
x_noisy = x; y_noisy = y; theta_noisy = theta;

% Store trajectory data
trajectory_est = [x_est; y_est];
trajectory_noisy = [x_noisy; y_noisy];

% Loop over the motion
for t = 0:T_s:t_end
    % Dynamic steering angle: start at pi/2 and decrease over time to 0
    beta = max(pi/2 * (1 - t/t_end), 0);
    
    % Add Gaussian noise to the angular velocity
    phi_dot_noisy = phi_dot + std_dev_phi * randn;
    
    % Calculate linear and angular velocities for estimated and noisy trajectories
    v_est = r * phi_dot * sin(beta);
    omega_est = -r * phi_dot * cos(beta) / L_2;
    v_noisy = r * phi_dot_noisy * sin(beta);
    omega_noisy = -r * phi_dot_noisy * cos(beta) / L_2;

    % Apply the second-order Runge-Kutta method for the orientation
    theta_mid = theta_est + (omega_est * T_s / 2);
    theta_mid_noisy = theta_noisy + (omega_noisy * T_s / 2);
    
    % Update the estimated and noisy positions
    x_est = x_est + T_s * v_est * cos(theta_mid);
    y_est = y_est + T_s * v_est * sin(theta_mid);
    theta_est = theta_est + T_s * omega_est;
    
    x_noisy = x_noisy + T_s * v_noisy * cos(theta_mid_noisy);
    y_noisy = y_noisy + T_s * v_noisy * sin(theta_mid_noisy);
    theta_noisy = theta_noisy + T_s * omega_noisy;
    
    % Append to the trajectories
    trajectory_est = [trajectory_est, [x_est; y_est]];
    trajectory_noisy = [trajectory_noisy, [x_noisy; y_noisy]];
end

% Plot the estimated and noisy trajectories
plot(trajectory_est(1,:), trajectory_est(2,:), 'Color', 'blue', 'LineWidth', 2); hold on;
plot(trajectory_noisy(1,:), trajectory_noisy(2,:), 'Color', [1, 0.5, 0], 'LineWidth', 2);

xlabel('X position');
ylabel('Y position');
title('Three-Wheel Forklift Trajectory with and without Gaussian Noise');
legend('Estimated Trajectory (without noise)', 'Noisy Trajectory (with noise)');
grid on;
