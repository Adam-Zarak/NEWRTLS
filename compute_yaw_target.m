function [yaw_target, distance_to_target] = compute_yaw_target(current_x, current_y, target_x, target_y)
    [distance, bearing] = Euclidean_distance(current_x, current_y, target_x, target_y);
    yaw_target = bearing;
    distance_to_target = distance;
end
