% function y_observed = observationModel(Vx, Vy, Vz, quat0, quat1, quat2, quat3)
% C = rotationMatrix(quat0, quat1, quat2, quat3);
% y_observed = C*[Vx; Vy; Vz];
% end
function y_observed = observationModel(Vx, Vy, Vz, quat0, quat1, quat2, quat3)
C = rotationMatrix(quat0, quat1, quat2, quat3);
% C body to world
y_observed = C*[Vx; Vy; Vz];
end

%% here Vx, Vy, Vz can be passed in different value (like x ,y ,z)