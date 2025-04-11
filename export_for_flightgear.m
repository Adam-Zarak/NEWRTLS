function export_for_flightgear(csv_filename, output_filename)
    % Load telemetry log
    data = readmatrix(csv_filename, 'NumHeaderLines', 1);

    % Extract relevant columns
    time = data(:,1);
    yaw_deg = data(:,2);      % Heading
    x_pos_ft = data(:,6);
    y_pos_ft = data(:,7);
    alt_ft = data(:,8);

    % Set constants
    alt_m = alt_ft * 0.3048 + 60; % Convert ft → m
    roll = zeros(size(time)); % Assume 0 for now
    pitch = zeros(size(time));

    % Reference origin (arbitrary lat/lon to place in FlightGear)
    origin_lat = 28.6024;     % UCF approx
    origin_lon = -81.2001;

    % Convert ft → meters
    x_m = x_pos_ft * 0.3048;
    y_m = y_pos_ft * 0.3048;

    % Calculate lat/lon offsets using simple equirectangular projection
    earth_radius_m = 6371000;  % Earth radius
    lat_deg = origin_lat + (y_m / earth_radius_m) * (180/pi);
    lon_deg = origin_lon + (x_m / (earth_radius_m * cosd(origin_lat))) * (180/pi);

    % Combine everything for FlightGear format
    fg_data = [time, lat_deg, lon_deg, alt_m, roll, pitch, yaw_deg];

    % Save to tab-delimited text file
    writematrix(fg_data, output_filename, 'Delimiter', 'tab');
end

