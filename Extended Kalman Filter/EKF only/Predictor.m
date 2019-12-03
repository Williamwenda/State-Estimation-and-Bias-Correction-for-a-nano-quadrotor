function P_check = Predictor(quat0, quat1, quat2, quat3, a_x, a_y, a_z, W_x, W_y, W_z, T, accel_var, om_var, EstimateCov)
% Returning the predicted covariances
F = Jacob_F(quat0, quat1, quat2, quat3, a_x, a_y, a_z, W_x, W_y, W_z, T);
Q = Q_prime(accel_var, om_var, T);
P_check = (F*EstimateCov*F')+Q;
end