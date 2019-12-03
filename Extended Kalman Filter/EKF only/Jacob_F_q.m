function linearMotion_q = Jacob_F_q(W_x, W_y, W_z, T)
der_q_pos = zeros(3,3);
der_q_vel = zeros(3,3);
W_skew = [0 -W_z W_y; W_z 0 -W_x; -W_y W_x 0];
der_q_quat = expm(T*W_skew);

linearMotion_q = [der_q_pos der_q_vel der_q_quat'];   
end