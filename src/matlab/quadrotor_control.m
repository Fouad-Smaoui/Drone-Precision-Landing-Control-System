% Quadrotor Control System Model
% This model implements the control algorithm for quadrotor landing on a moving platform

% System Parameters
m = 1.0;  % Mass of quadrotor (kg)
Ixx = 0.01;  % Moment of inertia around x-axis
Iyy = 0.01;  % Moment of inertia around y-axis
Izz = 0.02;  % Moment of inertia around z-axis
g = 9.81;  % Gravitational acceleration (m/s^2)
l = 0.2;  % Arm length (m)
kf = 1.0e-6;  % Thrust coefficient
km = 1.0e-7;  % Moment coefficient

% Control Parameters
Kp_pos = [2.0, 2.0, 2.0];  % Position control gains
Kd_pos = [1.0, 1.0, 1.0];  % Position derivative gains
Kp_att = [2.0, 2.0, 2.0];  % Attitude control gains
Kd_att = [1.0, 1.0, 1.0];  % Attitude derivative gains

% Moving Platform Parameters
platform_velocity = [0.5, 0.5, 0];  % Platform velocity (m/s)
platform_position = [0, 0, 0];  % Initial platform position

% Simulation Parameters
sim_time = 20;  % Simulation time (s)
step_size = 0.01;  % Step size (s)

% Create Simulink Model
mdl = 'quadrotor_control';
if ~bdIsLoaded(mdl)
    new_system(mdl);
end
open_system(mdl);

% Add blocks for quadrotor dynamics
add_block('simulink/Continuous/State-Space', [mdl '/Quadrotor Dynamics']);
add_block('simulink/Math Operations/Matrix Multiply', [mdl '/Control Allocation']);
add_block('simulink/Continuous/PID Controller', [mdl '/Position Controller']);
add_block('simulink/Continuous/PID Controller', [mdl '/Attitude Controller']);
add_block('simulink/Sources/Constant', [mdl '/Platform Position']);
add_block('simulink/Sources/Constant', [mdl '/Platform Velocity']);
add_block('simulink/Math Operations/Sum', [mdl '/Position Error']);
add_block('simulink/Sinks/To Workspace', [mdl '/Position Data']);
add_block('simulink/Sinks/To Workspace', [mdl '/Attitude Data']);
add_block('simulink/Sinks/To Workspace', [mdl '/Control Data']);

% Configure quadrotor dynamics
A = zeros(12, 12);
A(1:3, 4:6) = eye(3);
A(7:9, 10:12) = eye(3);
B = zeros(12, 4);
B(4:6, 1) = [0; 0; g];
B(10:12, 2:4) = diag([1/Ixx, 1/Iyy, 1/Izz]);
C = eye(12);
D = zeros(12, 4);

set_param([mdl '/Quadrotor Dynamics'], 'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D));

% Configure platform parameters
set_param([mdl '/Platform Position'], 'Value', mat2str(platform_position));
set_param([mdl '/Platform Velocity'], 'Value', mat2str(platform_velocity));

% Configure controllers
set_param([mdl '/Position Controller'], 'P', mat2str(Kp_pos), 'I', '[0 0 0]', 'D', mat2str(Kd_pos));
set_param([mdl '/Attitude Controller'], 'P', mat2str(Kp_att), 'I', '[0 0 0]', 'D', mat2str(Kd_att));

% Configure data logging
set_param([mdl '/Position Data'], 'VariableName', 'position_data');
set_param([mdl '/Attitude Data'], 'VariableName', 'attitude_data');
set_param([mdl '/Control Data'], 'VariableName', 'control_data');

% Connect blocks
add_line(mdl, 'Platform Position/1', 'Position Error/1');
add_line(mdl, 'Quadrotor Dynamics/1', 'Position Error/2');
add_line(mdl, 'Position Error/1', 'Position Controller/1');
add_line(mdl, 'Position Controller/1', 'Control Allocation/1');
add_line(mdl, 'Control Allocation/1', 'Quadrotor Dynamics/1');
add_line(mdl, 'Quadrotor Dynamics/1', 'Attitude Controller/1');
add_line(mdl, 'Attitude Controller/1', 'Control Allocation/2');
add_line(mdl, 'Quadrotor Dynamics/1', 'Position Data/1');
add_line(mdl, 'Quadrotor Dynamics/7', 'Attitude Data/1');
add_line(mdl, 'Control Allocation/1', 'Control Data/1');

% Save model
save_system(mdl);

% Run simulation
sim(mdl, sim_time);

% Plot results
figure('Name', 'Quadrotor Landing Simulation Results');

% Position tracking
subplot(3,1,1);
plot(position_data.time, position_data.signals.values(:,1:3));
hold on;
plot(position_data.time, platform_position(1) + platform_velocity(1)*position_data.time, '--');
plot(position_data.time, platform_position(2) + platform_velocity(2)*position_data.time, '--');
plot(position_data.time, platform_position(3) + platform_velocity(3)*position_data.time, '--');
legend('X', 'Y', 'Z', 'Platform X', 'Platform Y', 'Platform Z');
title('Position Tracking');
xlabel('Time (s)');
ylabel('Position (m)');
grid on;

% Attitude control
subplot(3,1,2);
plot(attitude_data.time, attitude_data.signals.values(:,1:3));
legend('Roll', 'Pitch', 'Yaw');
title('Attitude Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;

% Control inputs
subplot(3,1,3);
plot(control_data.time, control_data.signals.values);
legend('Thrust', 'Roll Cmd', 'Pitch Cmd', 'Yaw Cmd');
title('Control Inputs');
xlabel('Time (s)');
ylabel('Control Signal');
grid on; 