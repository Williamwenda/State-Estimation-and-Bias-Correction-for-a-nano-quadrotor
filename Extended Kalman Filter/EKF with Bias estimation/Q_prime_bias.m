function processNoise_bias = Q_prime_bias(accel_b_var, T, om_b_var, W_x_orig, W_y_orig, W_z_orig, bias_Wx, bias_Wy, bias_Wz,...
    quat0, quat1, quat2, quat3)
W_x= W_x_orig - bias_Wx; 
W_y = W_y_orig - bias_Wy;
W_z = W_z_orig - bias_Wz;
W_skew = [0 -W_z W_y; W_z 0 -W_x; -W_y W_x 0];
C = rotationMatrix(quat0, quat1, quat2, quat3);
gamma_two = (1/2)*(T^2)*eye(3,3)+(1/6)*(T^3)*W_skew+(1/24)*(T^4)*(W_skew^2);
der_b_xvq = [-(1/6)*(T^3)*accel_b_var*C -(1/2)*(T^2)*accel_b_var*C zeros(3,3); zeros(3,3) zeros(3,3) -(om_b_var)*(gamma_two)];
der_b = [T*accel_b_var zeros(3,3); zeros(3,3) T*om_b_var];
processNoise_bias = [der_b_xvq der_b];
end