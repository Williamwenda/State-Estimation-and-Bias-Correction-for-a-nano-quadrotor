function processNoise_omeg = Q_prime_omeg(om_var, T, om_b_var, W_x_orig, W_y_orig, W_z_orig, bias_Wx, bias_Wy, bias_Wz)
W_x= W_x_orig - bias_Wx; 
W_y = W_y_orig - bias_Wy;
W_z = W_z_orig - bias_Wz;
W_skew = [0 -W_z W_y; W_z 0 -W_x; -W_y W_x 0];
gamma_three = (1/6)*(T^3)*eye(3,3)+(1/24)*(T^4)*W_skew+(1/120)*(T^5)*(W_skew^2);
der_omega_b = (gamma_three+gamma_three')*om_b_var;
der_omeg_x = zeros(3,3);
der_omeg_v = zeros(3,3);
der_omeg_quat = (T*om_var)+der_omega_b;
der_omeg_ac_b = zeros(3,3);
gamma_two = (1/2)*(T^2)*eye(3,3)+(1/6)*(T^3)*W_skew+(1/24)*(T^4)*(W_skew^2);
der_omeg_b = -(gamma_two')*om_b_var;
processNoise_omeg = [der_omeg_x der_omeg_v der_omeg_quat der_omeg_ac_b der_omeg_b];
end