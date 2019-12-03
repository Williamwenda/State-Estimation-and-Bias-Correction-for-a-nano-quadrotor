function linearMotion_x = Jacob_F_x(quat0, quat1, quat2, quat3, a_x, a_y, a_z, T)
der_x_pos = eye(3,3);
der_x_vel = T*eye(3,3);
C = rotationMatrix(quat0, quat1, quat2, quat3);
%% C:body to world
accel_skew = [0 -a_z a_y; a_z 0 -a_x; -a_y a_x 0];
der_x_quat = -(1/2)*(T^2)*(C')*accel_skew;
linearMotion_x = [der_x_pos der_x_vel der_x_quat];
end