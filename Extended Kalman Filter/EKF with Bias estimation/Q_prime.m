function processNoise = Q_prime(accel_var, om_var, T, accel_b_var, om_b_var,...
    W_x_orig, W_y_orig, W_z_orig, bias_Wx, bias_Wy, bias_Wz, quat0, quat1, quat2, quat3)
processNoise_accel = Q_prime_accel(accel_var, T, accel_b_var, quat0, quat1, quat2, quat3);
processNoise_omeg = Q_prime_omeg(om_var, T, om_b_var, W_x_orig, W_y_orig, W_z_orig, bias_Wx, bias_Wy, bias_Wz);
processNoise_bias = Q_prime_bias(accel_b_var, T, om_b_var, W_x_orig, W_y_orig, W_z_orig, bias_Wx, bias_Wy, bias_Wz,...
    quat0, quat1, quat2, quat3);
processNoise = [processNoise_accel; processNoise_omeg; processNoise_bias];
end