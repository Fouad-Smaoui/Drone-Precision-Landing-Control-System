function control = pinn_interface(state)
% PINN_INTERFACE Interface between MATLAB and Python PINN controller
%   state: 12x1 vector containing [x, y, z, vx, vy, vz, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
%   control: 4x1 vector containing [thrust, roll_cmd, pitch_cmd, yaw_cmd]

% Convert state to JSON
state_json = jsonencode(state);

% Create HTTP request
url = 'http://localhost:5000/control';
options = weboptions('MediaType', 'application/json', 'RequestMethod', 'post');

try
    % Send request to Python server
    response = webwrite(url, state_json, options);
    
    % Parse response
    if strcmp(response.status, 'success')
        control = response.control;
    else
        error('PINN controller returned error: %s', response.message);
    end
catch e
    % Fallback to traditional control if PINN is not available
    warning('PINN controller not available, using traditional control');
    control = traditional_control(state);
end

end

function control = traditional_control(state)
% Traditional PID control as fallback
% Simple proportional control for demonstration
Kp = [2.0, 2.0, 2.0, 2.0];  % Control gains
control = Kp .* state(1:4)';  % Simple proportional control
end 