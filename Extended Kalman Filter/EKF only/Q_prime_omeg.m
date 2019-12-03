function processNoise_omeg = Q_prime_omeg(om_var, T)
der_omeg_x = zeros(3,3);
der_omeg_v = zeros(3,3);
der_omeg_quat = T*om_var;
processNoise_omeg = [der_omeg_x der_omeg_v der_omeg_quat];
end