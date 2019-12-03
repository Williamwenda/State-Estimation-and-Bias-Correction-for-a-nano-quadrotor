function [state_x, state_v, state_q]  = motionModel(x, y, z, V_x, V_y, V_z, quat0, quat1, quat2, quat3, a_x, a_y, a_z, W_x, W_y, W_z, T)
state_x = motionModel_x(x, y,z, V_x, V_y, V_z, quat0, quat1, quat2, quat3, a_x, a_y, a_z, T);
state_v = motionModel_v(V_x, V_y, V_z, quat0, quat1, quat2, quat3, a_x, a_y, a_z, T);
state_q = motionModel_q(quat0, quat1, quat2, quat3, W_x, W_y, W_z, T);
end