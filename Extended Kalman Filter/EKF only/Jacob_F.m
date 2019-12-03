function linearMotion = Jacob_F(quat0, quat1, quat2, quat3, a_x, a_y, a_z, W_x, W_y, W_z, T)
linearMotion_x = Jacob_F_x(quat0, quat1, quat2, quat3, a_x, a_y, a_z, T);
linearMotion_v = Jacob_F_v(quat0, quat1, quat2, quat3, a_x, a_y, a_z, T);
linearMotion_q = Jacob_F_q(W_x, W_y, W_z, T);
linearMotion = [linearMotion_x; linearMotion_v; linearMotion_q];
end