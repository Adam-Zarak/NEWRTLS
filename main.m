clc; clear;

% RTLS Flight Algorithm - Heading Based PD Control Only
if exist('telemetry_log.csv', 'file')
    delete('telemetry_log.csv');
end

% Initialize
current_x = randi([1000, 4000]);
current_y = randi([1000, 4000]);
target_x = 250;
target_y = 250;
dt = 0.1;
t = 0;
max_time = 150;
yaw = 0;
yaw_error_prev = 0;
altitude = 750;
velocity_z = -5;

% Controller Gains
Kp = 1.2;
Kd = 0.4;
max_angle = 15;

clear log_telemetry

while t < max_time && altitude > 0
    % Compute bearing and distance to target
    [target_bearing, distance_to_target] = Euclidean_distance(current_x, current_y, target_x, target_y);
    
    % Compute yaw error [-180, 180]
    yaw_error = mod(target_bearing - yaw + 180, 360) - 180;

    % PD Controller
    servo_cmd = pd_controller(yaw_error, yaw_error_prev, dt, Kp, Kd, max_angle);
    yaw_error_prev = yaw_error;

    % Simulate sensor & motion update
    [altitude, velocity_z, current_x, current_y, yaw] = get_sensor_data(servo_cmd, current_x, current_y, altitude, velocity_z, yaw, dt);

    % Log data
    log_telemetry(t, yaw, target_bearing, yaw_error, servo_cmd, current_x, current_y, altitude, velocity_z, distance_to_target);

    t = t + dt;
end
