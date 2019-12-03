function linearMotion_v = Jacob_F_v(quat0, quat1, quat2, quat3, a_x, a_y, a_z, T)
der_v_pos = zeros(3,3);
der_v_vel = eye(3,3);
C = rotationMatrix(quat0, quat1, quat2, quat3);
%% C body to world
accel_skew = [0 -a_z a_y; a_z 0 -a_x; -a_y a_x 0];
der_v_quat = -T*(C')*accel_skew;
linearMotion_v = [der_v_pos der_v_vel der_v_quat];
end