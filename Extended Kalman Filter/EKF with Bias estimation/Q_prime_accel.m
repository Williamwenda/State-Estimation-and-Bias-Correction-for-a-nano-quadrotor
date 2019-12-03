function processNoise_accel = Q_prime_accel(accel_var, T, accel_b_var, quat0, quat1, quat2, quat3)
C = rotationMatrix(quat0, quat1, quat2, quat3);
der_accel_xv = [(1/3)*(T^3)*accel_var+(1/20)*(T^5)*accel_b_var (1/2)*(T^2)*accel_var+(1/8)*(T^4)*accel_b_var;...
    (1/2)*(T^2)*accel_var+(1/8)*(T^4)*accel_b_var T*accel_var+(1/3)*(T^3)*accel_b_var];
der_accel_quat = zeros(6,3);
der_bias_accel = [(-1/6)*(T^3)*(C')*accel_b_var; (-1/2)*(T^2)*(C')*accel_b_var];
der_bias_omega = zeros(6,3);
processNoise_accel = [der_accel_xv der_accel_quat der_bias_accel der_bias_omega];
end