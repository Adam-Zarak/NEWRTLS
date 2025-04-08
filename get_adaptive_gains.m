function [Kp, Kd] = get_adaptive_gains(distance_to_target)
    % Returns adaptive gains based on distance to target

    if distance_to_target > 1000
        Kp = 2.0;
        Kd = 0.8;
    elseif distance_to_target > 300
        Kp = 1.5;
        Kd = 0.6;
    else
        Kp = 0.5;
        Kd = 0.2;
    end
end

