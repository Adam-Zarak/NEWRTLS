function [altitude, velocity_z, current_x, current_y, yaw] = ...
    get_sensor_data(servo_cmd, current_x, current_y, altitude, velocity_z, yaw, dt)

    % Parameters for realistic yaw change
    max_yaw_rate_deg = 120;  % max degrees/sec for full servo_cmd (±15)
    
    % Scale servo_cmd to yaw rate
    yaw_velocity = (servo_cmd / 15) * max_yaw_rate_deg;
    yaw = yaw + yaw_velocity * dt + randn * 0.01;  % include slight noise
    
    % Normalize yaw to [0, 360]
    yaw = mod(yaw, 360);

    % Simulate motion in direction of yaw
    step_size = 0.8;  % feet per timestep
    current_x = current_x + cosd(yaw) * step_size;
    current_y = current_y + sind(yaw) * step_size;

    % Update altitude
    altitude = altitude + velocity_z * dt;
end
