function [yaw_estimate, P] = adaptive_kalman(yaw_target, yaw_estimate_prev, P, Q_kf, R_kf)
    % Predict step
    P = P + Q_kf;

    % Compute Kalman Gain
    K = P / (P + R_kf);

    % Prevent NaN/Inf values
    if any(isnan(K(:))) || any(isinf(K(:)))
        K = 0.1;
    end

    % Update estimate
    yaw_estimate = yaw_estimate_prev + K * (yaw_target - yaw_estimate_prev);

    % Normalize yaw_estimate to [0, 360]
    yaw_estimate = mod(yaw_estimate, 360);

    % Update covariance
    P = (1 - K) * P;
end
