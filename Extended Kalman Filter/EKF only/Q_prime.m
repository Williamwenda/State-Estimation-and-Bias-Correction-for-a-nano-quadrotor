function processNoise = Q_prime(accel_var, om_var, T)
processNoise_accel = Q_prime_accel(accel_var, T);
processNoise_omeg = Q_prime_omeg(om_var, T);
processNoise = [processNoise_accel; processNoise_omeg];
end