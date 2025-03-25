function log_telemetry(t, yaw, yaw_target, yaw_error, servo_cmd, current_x, current_y, altitude, velocity_z, distance_to_target)
    % Define log file name
    log_filename = 'telemetry_log.csv';
    
    % Ensure headers are written **only once** at the start of the simulation
    persistent log_initialized;  % Keeps track if headers are written
    
    if isempty(log_initialized) || t == 0
        fid = fopen(log_filename, 'w'); % Overwrite file on first run
        fprintf(fid, 'Time,Yaw,Yaw_Target,Yaw_Error,Servo_Cmd,X_Position,Y_Position,Altitude,Velocity_Z,Distance_To_Target\n');
        log_initialized = true; % Mark headers as written
    else
        fid = fopen(log_filename, 'a'); % Append mode for subsequent writes
    end
    
    % Check if file opened correctly
    if fid == -1
        error('Could not open telemetry log file.');
    end
    
    % **Fix: Use `all(~isnan(...))` to check if all values are valid**
    if all(~isnan(yaw)) && all(~isnan(altitude)) && all(~isnan(distance_to_target))
        fprintf(fid, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n', ...
                t, yaw, yaw_target, yaw_error, servo_cmd, current_x, current_y, altitude, velocity_z, distance_to_target);
    end
    
    % Close the file
    fclose(fid);
end
