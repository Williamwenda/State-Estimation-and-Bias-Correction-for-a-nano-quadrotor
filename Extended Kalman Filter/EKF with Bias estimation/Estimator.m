function [K, P_hat] = Estimator(x,y,z, Vx, Vy, Vz, quat0, quat1, quat2, quat3, PredictedCov, ...
       X_var,Y_var,Z_var, Vx_var, Vy_var, Vz_var, choice)
% Returning the estimated covariances
G = Jacob_G(x,y,z, Vx, Vy, Vz, quat0, quat1, quat2, quat3, choice);
if choice == true
    R = R_prime(Vx_var, Vy_var, Vz_var);
else
% Usint X Y Z
    R = R_prime(X_var, Y_var, Z_var);
% Using Z Vx Vy
%       R = R_prime(Z_var, Vx_var, Vy_var);
end
K = (PredictedCov*(G'))/(G*PredictedCov*(G')+R);
iden_matrix = eye(15);
P_hat = (iden_matrix-(K*G))*PredictedCov;
end