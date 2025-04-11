function shifted_traj = shift_trajectory_to_iceland(traj_data)
% shift_trajectory_to_iceland - shifts your lat/lon/alt to Iceland for FlightGear

    % Iceland spawn target
    target_lat = 63.9850;
    target_lon = -22.6050;
    target_alt = 750; % feet

    % Original starting point
    orig_lat = traj_data(1, 2);
    orig_lon = traj_data(1, 3);
    orig_alt = traj_data(1, 4);

    % Compute the deltas
    lat_shift = target_lat - orig_lat;
    lon_shift = target_lon - orig_lon;
    alt_shift = target_alt - orig_alt;

    % Apply the shift
    shifted_traj = traj_data;
    shifted_traj(:, 2) = traj_data(:, 2) + lat_shift;
    shifted_traj(:, 3) = traj_data(:, 3) + lon_shift;
    shifted_traj(:, 4) = traj_data(:, 4) + alt_shift;
end


