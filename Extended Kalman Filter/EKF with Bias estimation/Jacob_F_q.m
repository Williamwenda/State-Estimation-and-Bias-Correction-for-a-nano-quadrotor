function linearMotion_q = Jacob_F_q(W_x_orig, W_y_orig, W_z_orig, T, bias_Wx, bias_Wy, bias_Wz)
W_x= W_x_orig - bias_Wx; 
W_y = W_y_orig - bias_Wy;
W_z = W_z_orig - bias_Wz;
der_q_pos = zeros(3,3);
der_q_vel = zeros(3,3);
W_skew = [0 -W_z W_y; W_z 0 -W_x; -W_y W_x 0];
der_q_quat = eye(3,3)+T*W_skew+(1/2)*(T^2)*(W_skew^2);
% der_q_quat = expm(T*W_skew);
der_b_accel = zeros(3,3);
der_b_omega = T*eye(3,3)+(1/2)*(T^2)*(W_skew)+(1/6)*(T^3)*(W_skew^2);
linearMotion_q = [der_q_pos der_q_vel der_q_quat' der_b_accel -der_b_omega'];   
end