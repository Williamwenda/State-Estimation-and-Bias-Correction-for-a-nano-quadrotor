function state_q = motionModel_q(quat0, quat1, quat2, quat3, W_x, W_y, W_z, T)
%% modified here
angel = sqrt((T*W_x)^2+(T*W_y)^2+(T*W_z)^2);
%angel = T*(sqrt((W_x^2)+(W_y^2)+(W_z^2)));
Eta = cos(0.5*angel);
Zeta_x = sin(0.5*angel)*(T*W_x/angel);
Zeta_y = sin(0.5*angel)*(T*W_y/angel);
Zeta_z = sin(0.5*angel)*(T*W_z/angel);
quat_input = [Eta, Zeta_x, Zeta_y, Zeta_z];
% quat_input = quaternion(Eta, Zeta_x, Zeta_y, Zeta_z)
% quat_prev = quaternion(quat0, quat1, quat2, quat3)
quat_prev = [quat0, quat1, quat2, quat3];
state_q = quatmultiply(quat_input,quat_prev);
state_q = state_q';
end