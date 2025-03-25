function servo_cmd = pd_controller(yaw_error, yaw_error_prev, dt, Kp, Kd, max_angle)
    % Pure PD controller
    d_error = (yaw_error - yaw_error_prev) / dt;
    servo_cmd = Kp * yaw_error + Kd * d_error;

    % Clamp to max allowable servo range
    servo_cmd = max(min(servo_cmd, max_angle), -max_angle);
end
