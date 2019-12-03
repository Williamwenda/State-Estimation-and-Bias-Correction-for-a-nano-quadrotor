function state_v = motionModel_v(V_x, V_y, V_z, quat0, quat1, quat2, quat3, a_x, a_y, a_z, T, bias_ax, bias_ay, bias_az)
g = [0; 0; 9.81];
state_pre_V = [V_x; V_y; V_z];
C = rotationMatrix(quat0, quat1, quat2, quat3);
accel_input = [a_x; a_y; a_z]+[bias_ax; bias_ay; bias_az];
% C body to world
state_v = state_pre_V+(T*C'*accel_input)-(T*g);
end