function state_x = motionModel_x(x, y,z, V_x, V_y, V_z, quat0, quat1, quat2, quat3, a_x, a_y, a_z, T)
g = [0; 0; 9.81];
state_pre_V = [V_x; V_y; V_z];
state_pre_X = [x; y; z];
C = rotationMatrix(quat0, quat1, quat2, quat3);
%% rotation matrix C should be Body to Inertial 
accel_input = [a_x; a_y; a_z];
state_x = state_pre_X+(T*state_pre_V)+((1/2)*(T^2)*C'*accel_input)-((1/2)*(T^2)*g);
end