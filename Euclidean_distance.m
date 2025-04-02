function [distance, bearing] = Euclidean_distance(x1, y1, x2, y2)
    % Compute Euclidean distance
    dx = x2 - x1;
    dy = y2 - y1;
    distance = sqrt(dx^2 + dy^2);

    % Compute bearing (heading) from current to target
    bearing = rad2deg(atan2(dy, dx));
    if bearing < 0
        bearing = bearing + 360;
    end

    % Display debug info
    disp(['Computing Distance: x1=', num2str(x1), ' y1=', num2str(y1),' dist=', num2str(distance)]);
end
