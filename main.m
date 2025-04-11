function main(disable_plots_flag)
%clearvars -except FINAL_DISTANCE results i;

if nargin < 1
    disable_plots_flag = false;
end


global FINAL_DISTANCE

FINAL_DISTANCE = NaN;  % <-- initialize early!


% Initialize data logging for 3D trajectory
x_log = [];
y_log = [];
z_log = [];

% RTLS Flight Algorithm - Heading Based PD Control Only
if exist('telemetry_log.csv', 'file')
    delete('telemetry_log.csv');
end

% Initialize
current_x = randi([1000, 2000]);
current_y = randi([1000, 2000]);
target_x = 250;
target_y = 250;
dt = 0.1;
t = 0;
max_time = 50;
yaw = randi([0,359]);
initial_yaw = yaw;
yaw_error_prev = 0;
altitude = 750;
velocity_z = -15;

% Wind configuration
wind_speed_mean = 0;        % Mean wind speed in ft/s
wind_speed_std = 1;         % Std deviation of wind gusts
wind_direction_mean = 0;    % Mean wind direction in radians (0 = +X direction)
wind_direction_std = pi/8;  % Directional variability

max_angle = 15;

clear log_telemetry

while t < max_time && altitude > 0
    % Compute bearing and distance to target
    [distance_to_target, target_bearing] = Euclidean_distance(current_x, current_y, target_x, target_y);
    target_bearing = mod(target_bearing, 360);

    % Compute yaw error [-180, 180]
    yaw_error = mod(target_bearing - yaw + 180, 360) - 180;

    % PD Controller
    % Adaptive PD Controller
    [Kp, Kd] = get_adaptive_gains(distance_to_target);
    servo_cmd = pd_controller(yaw_error, yaw_error_prev, dt, Kp, Kd, max_angle);

    yaw_error_prev = yaw_error;

    % Simulate sensor & motion update
    [altitude, velocity_z, current_x, current_y, yaw] = get_sensor_data(servo_cmd, current_x, current_y, altitude, velocity_z, yaw, dt);

    % Apply wind *after* movement is updated
    wind_speed = wind_speed_mean + randn * wind_speed_std;
    wind_direction = wind_direction_mean + randn * wind_direction_std;
    wind_dx = wind_speed * cos(wind_direction);
    wind_dy = wind_speed * sin(wind_direction);
    current_x = current_x + wind_dx * dt;
    current_y = current_y + wind_dy * dt;

    % Log trajectory data
    x_log(end+1) = current_x;
    y_log(end+1) = current_y;
    z_log(end+1) = altitude;

    % Log data
    log_telemetry(t, yaw, target_bearing, yaw_error, servo_cmd, current_x, current_y, altitude, velocity_z, distance_to_target);

    t = t + dt;

    %display data to command window
    if mod(t, 1) < dt
    fprintf('t=%.1fs | X=%.1f | Y=%.1f | Alt=%.1f | Yaw=%.1f | YawTarget=%.1f | Err=%.1f | Dist=%.1f\n', ...
        t, current_x, current_y, altitude, yaw, target_bearing, yaw_error, distance_to_target);
    end

end

% Store final distance globally (for Monte Carlo runner)
if exist('distance_to_target', 'var') && ~isempty(distance_to_target)
    FINAL_DISTANCE = distance_to_target;
else
    FINAL_DISTANCE = NaN; % simulation exited early
end

if ~disable_plots_flag
    figure;
% =======================
% 3D Trajectory Plot
% =======================
figure;

% Plot trajectory
plot3(x_log, y_log, z_log, 'b-', 'LineWidth', 2); hold on;

% Start position
plot3(x_log(1), y_log(1), z_log(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
text(x_log(1), y_log(1), z_log(1)+30, 'Start', 'Color', 'g', 'FontSize', 10);

% Target position
plot3(target_x, target_y, 0, 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(target_x, target_y, 30, 'Target', 'Color', 'r', 'FontSize', 10);

% Orientation arrow
arrow_length = 100;
quiver3(x_log(1), y_log(1), z_log(1), ...
        cosd(initial_yaw)*arrow_length, sind(initial_yaw)*arrow_length, 0, ...
        'k', 'LineWidth', 2, 'MaxHeadSize', 2);
text(x_log(1), y_log(1), z_log(1)+60, 'Initial Orientation', 'FontSize', 10);

% Draw 800 ft radius circle around target
r = 800;
theta = linspace(0, 2*pi, 100);
circle_x = target_x + r * cos(theta);
circle_y = target_y + r * sin(theta);
circle_z = zeros(size(circle_x));
fill3(circle_x, circle_y, circle_z, 'r', ...
      'FaceAlpha', 0.1, 'EdgeColor', 'r', 'LineStyle', '--');

% Annotate distance from final position
final_x = x_log(end);
final_y = y_log(end);
final_dist = sqrt((final_x - target_x)^2 + (final_y - target_y)^2);
plot3([final_x, target_x], [final_y, target_y], [0, 0], 'k--');
text((final_x + target_x)/2, (final_y + target_y)/2, 20, ...
     sprintf('Distance to Target: %.2f ft', final_dist), ...
     'HorizontalAlignment', 'center', 'Color', 'k', 'FontSize', 10);

% Formatting
xlabel('X Position (ft)');
ylabel('Y Position (ft)');
zlabel('Altitude (ft)');
title({'3D Flight Path of RTLS Rocket'; ''}, 'FontWeight', 'bold', 'FontSize', 14);
grid on;
axis equal;   
end

global FINAL_DISTANCE;

% Create traj_data matrix from simulation logs
% Format: [time, latitude, longitude, altitude, roll, pitch, heading]

time = (0:length(x_log)-1)' * dt;

% Convert from feet to degrees (local offsets from Iceland)
% 1 degree latitude ≈ 364,000 ft
% 1 degree longitude ≈ 288,200 ft at ~64°N
lat0 = 63.9850;
lon0 = -22.6050;

latitude = lat0 + y_log(:) / 364000;
longitude = lon0 + x_log(:) / 288200;
altitude = z_log(:);     % Already in feet
pitch = -5 + 0.1 * time; % simulate gradual pitch-down
roll = 2 * sin(time);    % oscillating roll
heading = initial_yaw * ones(size(time));

traj_data = [time, latitude, longitude, altitude, roll, pitch, heading];

traj_data(:, 4) = traj_data(:, 4) * 0.3048;  % convert ft to m

% Export
writematrix(traj_data, 'flightgear_trajectory.txt', 'Delimiter', 'tab');
fprintf('FlightGear trajectory exported to flightgear_trajectory.txt\n');



