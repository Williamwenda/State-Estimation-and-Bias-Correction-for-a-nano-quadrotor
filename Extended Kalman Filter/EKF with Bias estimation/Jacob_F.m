function linearMotion = Jacob_F(quat0, quat1, quat2, quat3, a_x, a_y, a_z, W_x, W_y, W_z, T, bias_ax, bias_ay, bias_az, bias_Wx, bias_Wy, bias_Wz)
linearMotion_x = Jacob_F_x(quat0, quat1, quat2, quat3, a_x, a_y, a_z, T, bias_ax, bias_ay, bias_az);
linearMotion_v = Jacob_F_v(quat0, quat1, quat2, quat3, a_x, a_y, a_z, T, bias_ax, bias_ay, bias_az);
linearMotion_q = Jacob_F_q(W_x, W_y, W_z, T, bias_Wx, bias_Wy, bias_Wz);
b_accel = [eye(3,3); zeros(3,3)];
b_omega = [zeros(3,3); eye(3,3)];
bias = [zeros(6,9) b_accel b_omega];
linearMotion = [linearMotion_x; linearMotion_v; linearMotion_q; bias];
end