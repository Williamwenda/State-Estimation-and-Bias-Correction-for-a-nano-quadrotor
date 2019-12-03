% function linearObservation = Jacob_G(Vx, Vy, Vz, quat0, quat1, quat2, quat3)
% C = rotationMatrix(quat0, quat1, quat2, quat3);
% v_quat = C*[Vx; Vy; Vz];
% observ_q_skew = [0 -v_quat(3) v_quat(2); v_quat(3) 0 -v_quat(1); -v_quat(2) v_quat(1) 0];
% oserv_x = zeros(3,3);
% linearObservation = [oserv_x C observ_q_skew];
% end
function linearObservation = Jacob_G(x,y,z, Vx, Vy, Vz, quat0, quat1, quat2, quat3, choice)
C = rotationMatrix(quat0, quat1, quat2, quat3);
if choice == true
    %% modified here
    v_quat = C*[Vx; Vy; Vz]; % C body to world
    observ_q_skew = [0 -v_quat(3) v_quat(2); v_quat(3) 0 -v_quat(1); -v_quat(2) v_quat(1) 0];
    observ_x = zeros(3,3);
    linearObservation = [observ_x C observ_q_skew];
else
%% Using X Y Z
    v_quat = C*[x; y; z];
    observ_q_skew = [0 -v_quat(3) v_quat(2); v_quat(3) 0 -v_quat(1); -v_quat(2) v_quat(1) 0];  
    observ_x = C;
    observ_V = zeros(3,3);
    linearObservation = [observ_x observ_V observ_q_skew];

%% Using Zrange, Vx, Vy
%       X_quat=C*[x; y; z];
%       V_quat=C*[Vx; Vy; Vz];
%       Quat=[X_quat(3); V_quat(1); V_quat(2)];
%       observ_q_skew=[0 -Quat(3) Quat(2); Quat(3) 0 -Quat(1); -Quat(2) Quat(1) 0];
%       observ_X = [C(3,1) C(3,2) C(3,3); 0 0 0; 0 0 0];
%       observ_V = [0  0  0; C(1,1) C(1,2) C(1,3); C(2,1)  C(2,2)  C(2,3)];
%       linearObservation = [observ_X observ_V observ_q_skew];
end
end