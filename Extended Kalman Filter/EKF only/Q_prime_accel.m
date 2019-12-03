function processNoise_accel = Q_prime_accel(accel_var, T)
der_accel_xv = [(1/3)*(T^3)*accel_var (1/2)*(T^2)*accel_var; (1/2)*(T^2)*accel_var T*accel_var];
der_accel_quat = zeros(6,3);
processNoise_accel = [der_accel_xv der_accel_quat];
end