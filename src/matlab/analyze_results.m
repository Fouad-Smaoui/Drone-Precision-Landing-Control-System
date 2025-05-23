% Analyze simulation results and generate performance metrics

% Load simulation data
load('simulation_results.mat');

% Calculate performance metrics
% 1. Position tracking error
position_error = position_data.signals.values - ...
    [platform_position(1) + platform_velocity(1)*position_data.time, ...
     platform_position(2) + platform_velocity(2)*position_data.time, ...
     platform_position(3) + platform_velocity(3)*position_data.time];

rmse_position = sqrt(mean(position_error.^2));
max_position_error = max(abs(position_error));

% 2. Attitude control performance
rmse_attitude = sqrt(mean(attitude_data.signals.values.^2));
max_attitude_error = max(abs(attitude_data.signals.values));

% 3. Control effort
control_effort = sum(abs(control_data.signals.values), 1);
total_control_effort = sum(control_effort);

% 4. Settling time (time to reach within 5% of final value)
settling_threshold = 0.05;
settling_time = zeros(1,3);
for i = 1:3
    final_value = position_error(end,i);
    threshold = abs(final_value * settling_threshold);
    settling_idx = find(abs(position_error(:,i)) <= threshold, 1);
    if ~isempty(settling_idx)
        settling_time(i) = position_data.time(settling_idx);
    else
        settling_time(i) = inf;
    end
end

% Display results
fprintf('Performance Metrics:\n');
fprintf('-------------------\n');
fprintf('Position Control:\n');
fprintf('  RMSE (X,Y,Z): [%.3f, %.3f, %.3f] m\n', rmse_position);
fprintf('  Max Error (X,Y,Z): [%.3f, %.3f, %.3f] m\n', max_position_error);
fprintf('  Settling Time (X,Y,Z): [%.2f, %.2f, %.2f] s\n', settling_time);

fprintf('\nAttitude Control:\n');
fprintf('  RMSE (Roll,Pitch,Yaw): [%.3f, %.3f, %.3f] rad\n', rmse_attitude);
fprintf('  Max Error (Roll,Pitch,Yaw): [%.3f, %.3f, %.3f] rad\n', max_attitude_error);

fprintf('\nControl Effort:\n');
fprintf('  Total Control Effort: %.3f\n', total_control_effort);
fprintf('  Individual Control Efforts: [%.3f, %.3f, %.3f, %.3f]\n', control_effort);

% Plot additional analysis
figure('Name', 'Performance Analysis');

% Position error over time
subplot(2,2,1);
plot(position_data.time, position_error);
title('Position Error Over Time');
xlabel('Time (s)');
ylabel('Error (m)');
legend('X', 'Y', 'Z');
grid on;

% Attitude error over time
subplot(2,2,2);
plot(attitude_data.time, attitude_data.signals.values);
title('Attitude Error Over Time');
xlabel('Time (s)');
ylabel('Error (rad)');
legend('Roll', 'Pitch', 'Yaw');
grid on;

% Control effort over time
subplot(2,2,3);
plot(control_data.time, abs(control_data.signals.values));
title('Control Effort Over Time');
xlabel('Time (s)');
ylabel('Effort');
legend('Thrust', 'Roll', 'Pitch', 'Yaw');
grid on;

% 3D trajectory
subplot(2,2,4);
plot3(position_data.signals.values(:,1), ...
      position_data.signals.values(:,2), ...
      position_data.signals.values(:,3), 'b-');
hold on;
plot3(platform_position(1) + platform_velocity(1)*position_data.time, ...
      platform_position(2) + platform_velocity(2)*position_data.time, ...
      platform_position(3) + platform_velocity(3)*position_data.time, 'r--');
title('3D Trajectory');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
legend('Quadrotor', 'Platform');
grid on;
view(3); 