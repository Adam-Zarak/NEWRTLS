function [altitude, velocity_z, current_x, current_y, yaw] = get_sensor_data(servo_cmd, current_x, current_y, altitude, velocity_z, yaw)
    % Simulate yaw update with servo input + noise
    yaw = yaw + servo_cmd * 0.3 + randn * 0.01;

    % Move forward in direction of yaw
    step_size = 0.3;
    current_x = current_x + cosd(yaw) * step_size;
    current_y = current_y + sind(yaw) * step_size;

    % Update altitude and simulate descent
    % Descent update
    altitude = altitude + velocity_z * 0.1; % update altitude over time step

    yaw = mod(yaw, 360);  % wrap between 0–360

end
