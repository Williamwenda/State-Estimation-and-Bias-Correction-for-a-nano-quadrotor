function linearMotion_v = Jacob_F_v(quat0, quat1, quat2, quat3, a_x_orig, a_y_orig, a_z_orig, T, bias_ax, bias_ay, bias_az)
a_x = a_x_orig - bias_ax;
a_y = a_y_orig - bias_ay;
a_z = a_z_orig - bias_az;
der_v_pos = zeros(3,3);
der_v_vel = eye(3,3);
C = rotationMatrix(quat0, quat1, quat2, quat3);
%% C body to world
accel_skew = [0 -a_z a_y; a_z 0 -a_x; -a_y a_x 0];
der_v_quat = -T*(C')*accel_skew;
der_b_accel = -T*(C');
der_b_omega = zeros(3,3);
linearMotion_v = [der_v_pos der_v_vel der_v_quat der_b_accel der_b_omega];
end