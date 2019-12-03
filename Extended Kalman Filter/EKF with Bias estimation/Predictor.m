function P_check = Predictor(quat0, quat1, quat2, quat3, a_x, a_y, a_z, W_x, W_y, W_z, T, accel_var, om_var, EstimateCov,...
    bias_ax, bias_ay, bias_az, bias_Wx, bias_Wy, bias_Wz, accel_b_var, om_b_var)
% Returning the predicted covariances
F = Jacob_F(quat0, quat1, quat2, quat3, a_x, a_y, a_z, W_x, W_y, W_z, T, bias_ax, bias_ay, bias_az, bias_Wx, bias_Wy, bias_Wz);
Q = Q_prime(accel_var, om_var, T, accel_b_var, om_b_var, W_x, W_y, W_z, bias_Wx, bias_Wy, bias_Wz, quat0, quat1, quat2, quat3);
P_check = (F*EstimateCov*F')+Q;
end