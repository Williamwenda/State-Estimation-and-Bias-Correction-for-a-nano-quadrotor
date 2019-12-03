function [state_x, state_v, state_q, b_accel, b_omega]  = motionModel(x, y, z, V_x, V_y, V_z, quat0, quat1, quat2, quat3,...
    a_x, a_y, a_z, W_x, W_y, W_z, T, bias_ax, bias_ay, bias_az, bias_Wx, bias_Wy, bias_Wz)
state_x = motionModel_x(x, y,z, V_x, V_y, V_z, quat0, quat1, quat2, quat3, a_x, a_y, a_z, T, bias_ax, bias_ay, bias_az);
state_v = motionModel_v(V_x, V_y, V_z, quat0, quat1, quat2, quat3, a_x, a_y, a_z, T, bias_ax, bias_ay, bias_az);
state_q = motionModel_q(quat0, quat1, quat2, quat3, W_x, W_y, W_z, T, bias_Wx, bias_Wy, bias_Wz);
b_accel = [bias_ax; bias_ay; bias_az];
b_omega = [bias_Wx; bias_Wy; bias_Wz];
end