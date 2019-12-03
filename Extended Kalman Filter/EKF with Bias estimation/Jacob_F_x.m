function linearMotion_x = Jacob_F_x(quat0, quat1, quat2, quat3, a_x_orig, a_y_orig, a_z_orig, T, bias_ax, bias_ay, bias_az)
a_x = a_x_orig - bias_ax;
a_y = a_y_orig - bias_ay;
a_z = a_z_orig - bias_az;
der_x_pos = eye(3,3);
der_x_vel = T*eye(3,3);
C = rotationMatrix(quat0, quat1, quat2, quat3);
%% C:body to world
accel_skew = [0 -a_z a_y; a_z 0 -a_x; -a_y a_x 0];
der_x_quat = -(1/2)*(T^2)*(C')*accel_skew;
der_b_accel = -(1/2)*(T^2)*(C');
der_b_omega = zeros(3,3);
linearMotion_x = [der_x_pos der_x_vel der_x_quat der_b_accel der_b_omega];
end