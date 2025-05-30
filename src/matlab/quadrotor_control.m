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
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if exist([mdl '.slx'], 'file')
    delete([mdl '.slx']);
end
new_system(mdl);
open_system(mdl);

% Add blocks for quadrotor dynamics
add_block('simulink/Continuous/State-Space', [mdl '/Quadrotor Dynamics'], 'Position', [100 100 250 250]);
% Set number of inputs to 4 for thrust and three torques
add_block('simulink/Math Operations/Product', [mdl '/Control Allocation'], 'Position', [400 100 500 200], 'Inputs', '4', 'Multiplication', 'Matrix simulation');
add_block('simulink/Continuous/PID Controller', [mdl '/Position Controller'], 'Position', [250 50 350 100]);
add_block('simulink/Continuous/PID Controller', [mdl '/Attitude Controller'], 'Position', [250 200 350 250]);
add_block('simulink/Sources/Constant', [mdl '/Platform Position'], 'Position', [50 50 100 100]);
add_block('simulink/Sources/Constant', [mdl '/Platform Velocity'], 'Position', [50 150 100 200]);
add_block('simulink/Math Operations/Sum', [mdl '/Position Error'], 'Inputs', '+-', 'Position', [150 50 200 100]);
% Configure To Workspace blocks to save data as StructureWithTime
add_block('simulink/Sinks/To Workspace', [mdl '/Position Data'], 'Position', [600 50 650 100], 'SaveFormat', 'StructureWithTime');
add_block('simulink/Sinks/To Workspace', [mdl '/Attitude Data'], 'Position', [600 200 650 250], 'SaveFormat', 'StructureWithTime');
add_block('simulink/Sinks/To Workspace', [mdl '/Control Data'], 'Position', [600 125 650 175], 'SaveFormat', 'StructureWithTime');

% Add Selector block to extract position (states 1-3) from state vector
add_block('simulink/Signal Routing/Selector', [mdl '/Position Selector'], ...
    'IndexMode', 'One-based', ...
    'NumberOfDimensions', '1', ...
    'IndexOptionArray', {'Index vector (dialog)'}, ...
    'IndexParamArray', {'[1 2 3]'}, ...
    'InputPortWidth', '-1', ...
    'Position', [300 100 350 150]);

% Add Selector block to extract the attitude states (7,8,9) from the 12-vector
add_block('simulink/Signal Routing/Selector', [mdl '/Attitude Selector'], ...
    'Position', [300 200 350 250], ...
    'IndexMode', 'One-based', ...
    'NumberOfDimensions', '1', ...
    'IndexOptionArray', {'Index vector (dialog)'}, ...
    'IndexParamArray', {'[7 8 9]'}, ...
    'InputPortWidth', '-1'); % inherit the full 12-vector

% Add 3x1 constant for the desired attitude (here [0 0 0])
add_block('simulink/Sources/Constant', [mdl '/Attitude Reference'], ...
    'Position', [250 150 300 180]);
set_param([mdl '/Attitude Reference'], 'Value', '[0 0 0]');

% Attitude error = (reference – actual)
add_block('simulink/Math Operations/Sum', [mdl '/Attitude Error'], ...
    'Inputs', '+-', ...
    'Position', [350 150 400 200]);

% Add Selector for the thrust command (from Position Controller output)
add_block('simulink/Signal Routing/Selector', [mdl '/Thrust Selector'], ...
    'Position',[350 60 380 90], ...
    'IndexMode','One-based', ...
    'NumberOfDimensions','1', ...
    'IndexOptionArray',{'Index vector (dialog)'}, ...
    'IndexParamArray', {'[3]'}, ... % Assuming thrust is the 3rd output of Position Controller
    'InputPortWidth','-1');

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

% Select each of the three torque scalars out of the 3×1 attitude‐PID output
torque_idxs = [1 2 3]; % Assuming roll, pitch, yaw torque commands are outputs 1, 2, 3
for i = 1:3
    relName = sprintf('Torque%d Selector',i);
    fullName = [mdl '/' relName];
    add_block('simulink/Signal Routing/Selector', fullName, ...
        'Position',[350 110+(i-1)*50 380 140+(i-1)*50], ...
        'IndexMode','One-based', ...
        'NumberOfDimensions','1', ...
        'IndexOptionArray',{'Index vector (dialog)'}, ...
        'IndexParamArray',{sprintf('[%d]',torque_idxs(i))}, ...
        'InputPortWidth','-1');
    % wire Attitude PID → this selector → Mux port (i+1)
    add_line(mdl,'Attitude Controller/1',[relName '/1'],'autorouting','on');
    add_line(mdl,[relName '/1'],sprintf('Control Allocation/%d',i+1),'autorouting','on');
end

% Connect blocks
add_line(mdl, 'Platform Position/1', 'Position Error/1');
% Connect output of Quadrotor Dynamics to Position Selector input
add_line(mdl, 'Quadrotor Dynamics/1', 'Position Selector/1');
% Connect output of Position Selector to Position Error input 2
add_line(mdl, 'Position Selector/1', 'Position Error/2');
add_line(mdl, 'Position Error/1', 'Position Controller/1');
% Connect Position Controller output to Thrust Selector input
add_line(mdl,'Position Controller/1','Thrust Selector/1');
% Connect Thrust Selector output to Control Allocation input 1
add_line(mdl,'Thrust Selector/1','Control Allocation/1');

% Connect Quadrotor Dynamics output to Attitude Selector input
add_line(mdl, 'Quadrotor Dynamics/1',         'Attitude Selector/1');
add_line(mdl, 'Attitude Reference/1',         'Attitude Error/1');
add_line(mdl, 'Attitude Selector/1',          'Attitude Error/2');
add_line(mdl, 'Attitude Error/1',             'Attitude Controller/1');

% Connect Control Allocation output to Quadrotor Dynamics input
add_line(mdl, 'Control Allocation/1', 'Quadrotor Dynamics/1');

% Connect Quadrotor Dynamics output to To Workspace blocks
add_line(mdl, 'Quadrotor Dynamics/1', 'Position Data/1');
add_line(mdl, 'Quadrotor Dynamics/1', 'Attitude Data/1'); % Note: Attitude Data block might need attitude specific signals, reconsider this connection later if needed
add_line(mdl, 'Control Allocation/1', 'Control Data/1');

% Save model
save_system(mdl);

% Run simulation
sim(mdl, sim_time);

% Save simulation results for external analysis
save('simulation_results.mat', 'tout', 'position_data', 'attitude_data', 'control_data');

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
plot(attitude_data.time, attitude_data.signals.values(:,1:3)); % Assuming attitude_data saves a structure with time and signals
legend('Roll', 'Pitch', 'Yaw');
title('Attitude Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;

% Control inputs
subplot(3,1,3);
plot(control_data.time, control_data.signals.values); % Assuming control_data saves a structure with time and signals
legend('Thrust', 'Roll Cmd', 'Pitch Cmd', 'Yaw Cmd');
title('Control Inputs');
xlabel('Time (s)');
ylabel('Control Signal');
grid on; 