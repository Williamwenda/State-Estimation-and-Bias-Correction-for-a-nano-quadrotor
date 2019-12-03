%% choice == true , use velocity as the measurement
%  choice == false, use position as the measurement
function [x_hat, P_hat] = Crazyflie_EKF(choice, result_plot)
close all
clc

load('Data_04.mat','State0','P0','Ax','Ay','Az','Acc_variance','Wx','Wy','Wz','Omega_variance',...
    'Flowdeck_Vx','Flowdeck_Vy','Flowdeck_Vz','Flowdeck_vx_variance','Flowdeck_vy_variance',...
    'Flowdeck_vz_variance','time','T','X_true','Y_true','Z_true','Vx_true','Vy_true','Vz_true',...
    'Xinte','Yinte','Zrange','Var_X','Var_Y','Var_zrange')

x_0 = State0;
P_0 = P0;
a_x = Ax;
a_y = Ay;
a_z = Az;
W_x = 0.05*Wx;
W_y = 0.05*Wy;
W_z = 0.05*Wz;

% Test for gyro data
% W_x(1:350)=0;
% W_y(1:350)=0;
% Test for acc data
% a_x(1:450)=0;
% a_y(1:450)=0;

%% test for the noise in input acc data
% ax_true=zeros(1,length(time));
% ay_true=zeros(1,length(time));
% az_true=zeros(1,length(time));
% for i=2:length(time)
%     ax_true(i)=(Vx_true(i)-Vx_true(i-1))/T;
%     ay_true(i)=(Vy_true(i)-Vy_true(i-1))/T;
%     az_true(i)=(Vz_true(i)-Vz_true(i-1))/T+9.81;
% end
%  ax_true=lowpass(ax_true,50,1000);
%  ay_true=lowpass(ay_true,50,1000);
%  az_true=lowpass(az_true,50,1000);
%  
%  a_x=ax_true;
%  a_y=ay_true;
%  a_z=az_true;

accel_var = Acc_variance;
om_var = Omega_variance;
%% Here test for increase input data variance
accel_var = accel_var*450;
om_var  =  om_var*450;

Vx_var = Flowdeck_vx_variance;
Vy_var = Flowdeck_vy_variance;
Vz_var = Flowdeck_vz_variance;
Z_var = Var_zrange;
%% Modified here
% Add X_measure Y_measure
X_meas=Xinte;
Y_meas=Yinte;
% Add X_var, Y_var
X_var=Var_X;
Y_var=Var_Y;
%% End


Vx = Flowdeck_Vx;
Vy = Flowdeck_Vy;
Vz = Flowdeck_Vz;
% Vz = Vz_true;
t = time;

x_true = X_true;
y_true = Y_true;
z_true = Z_true;
% Vx_true = Vx_true;
% Vy_true = Vy_true;
% Vz_true = Vz_true;


% % Sampling Time (T) is 0.1
% T=0.1;

% P_check and P_hat are covariance matrices which are 9x9 at each timestep
P_check = zeros(9,9,length(t));
P_hat = zeros(9,9,length(t));

% x_check and x_hat are predicted and estimated vectors which are 10x1 at each timestep
x_check = zeros(length(t),10);
x_hat = zeros(length(t),10);

% intial values for estimator mean and covariance
P_hat(:,:,1) = P_0;
x_hat(1,:) = x_0;

% Save roll, pitch and yaw
roll=zeros(length(t),1);
pitch=zeros(length(t),1);
yaw = zeros(length(t),1);

for i = 2:length(t)
    P_check(:,:,i) = Predictor(x_hat(i-1,7), x_hat(i-1,8), x_hat(i-1,9), x_hat(i-1,10),...
        a_x(i), a_y(i), a_z(i), W_x(i), W_y(i), W_z(i), T, accel_var, om_var,...
        P_hat(:,:,i-1));

    [x_check(i,1:3), x_check(i,4:6), x_check(i,7:10)]  = motionModel(x_hat(i-1,1), x_hat(i-1,2), x_hat(i-1,3),...
        x_hat(i-1,4), x_hat(i-1,5), x_hat(i-1,6), x_hat(i-1,7), x_hat(i-1,8), x_hat(i-1,9), x_hat(i-1,10),...
        a_x(i), a_y(i), a_z(i), W_x(i), W_y(i), W_z(i), T);
    
 %Modified Estimator part to handle x y z in to matrix G
    [K, P_hat(:,:,i)] = Estimator(x_check(i,1),x_check(i,2),x_check(i,3), x_check(i,4), x_check(i,5), x_check(i,6),...
        x_check(i,7), x_check(i,8), x_check(i,9), x_check(i,10), P_check(:,:,i),...
        X_var, Y_var, Z_var, Vx_var, Vy_var, Vz_var, choice);
    
    if choice == true
        y_meas = [Vx(i); Vy(i); Vz(i)];
        y_observed = observationModel(x_check(i,4), x_check(i,5), x_check(i,6),...
            x_check(i,7), x_check(i,8), x_check(i,9), x_check(i,10));
    else
%         y_meas = [Zrange(i); Vx(i); Vy(i)];
%         y_observed = observationModel(x_check(i,3), x_check(i,4), x_check(i,5),...
%             x_check(i,7), x_check(i,8), x_check(i,9), x_check(i,10));

%% Modified here, send  X_measure,Y_measure as the measurements 
          y_meas = [X_meas(i);Y_meas(i);Zrange(i)];
          
          y_observed = observationModel(x_check(i,1), x_check(i,2), x_check(i,3),...
             x_check(i,7), x_check(i,8), x_check(i,9), x_check(i,10));
          
    end
    
    correction = (K*(y_meas-y_observed));
    
    x_predicted = x_check(i,:)';
    x_hat(i,1:6) = x_predicted(1:6)+correction(1:6);
    % update attitude    
    angel = (sqrt((correction(7)^2)+(correction(8)^2)+(correction(9)^2)));
    Eta = cos(0.5*angel);
    Zeta_x = sin(0.5*angel)*(correction(7)/angel);
    Zeta_y = sin(0.5*angel)*(correction(8)/angel);
    Zeta_z = sin(0.5*angel)*(correction(9)/angel);
    quat_correction = [Eta, Zeta_x, Zeta_y, Zeta_z];
    quat_check = x_check(i,7:10);
    q_hat = quatmultiply(quat_correction,quat_check);
    x_hat(i,7:10) = q_hat;
    %% Calculate roll, pitch, yaw for animation 
   
    [roll(i),pitch(i),yaw(i)]=quat2angle(q_hat);
end

% plotting the result
if (result_plot == true)
    sigma_x = [];
    sigma_y = [];
    sigma_z = [];
    sigma_vx = [];
    sigma_vy = [];
    sigma_vz = [];
    for i=1:length(P_hat)
        P_hat_x = sqrt(P_hat(1,1,i));
        sigma_x = [sigma_x; P_hat_x];
        P_hat_y = sqrt(P_hat(2,2,i));
        sigma_y = [sigma_y; P_hat_y];
        P_hat_z = sqrt(P_hat(3,3,i));
        sigma_z = [sigma_z; P_hat_z];
        P_hat_vx = sqrt(P_hat(4,4,i));
        sigma_vx = [sigma_vx; P_hat_vx];
        P_hat_vy = sqrt(P_hat(5,5,i));
        sigma_vy = [sigma_vy; P_hat_vy];
        P_hat_vz = sqrt(P_hat(6,6,i));
        sigma_vz = [sigma_vz; P_hat_vz];
    end
    epsilon = 0.01;
    % Error
    error_x = x_hat(:,1) - x_true;
    error_y = x_hat(:,2) - y_true;
    error_z = x_hat(:,3) - z_true;
    error_vx= x_hat(:,4) - Vx_true;
    error_vy= x_hat(:,5) - Vy_true;
    error_vz= x_hat(:,6) - Vz_true;
    
    % 3 sigma band
    x_p_envlope =  3*sigma_x;
    x_n_envlope = -3*sigma_x;
    y_p_envlope =  3*sigma_y;
    y_n_envlope = -3*sigma_y;
    z_p_envlope =  3*sigma_z;
    z_n_envlope = -3*sigma_z;
    vx_p_envlope = 3*sigma_vx;
    vx_n_envlope = -3*sigma_vx;
    vy_p_envlope = 3*sigma_vy;
    vy_n_envlope = -3*sigma_vy;
    vz_p_envlope = 3*sigma_vz;
    vz_n_envlope = -3*sigma_vz;
    %%
    
    % Be careful about which data you wanna to save, need to change the
    % name of the .mat file
    
    
    
%     save('Results/V_measture.mat','t','x_true','y_true','z_true','Vx_true','Vy_true','Vz_true',...
%         'x_hat','error_x','error_y','error_z','error_vx','error_vy','error_vz',...
%         'x_p_envlope','x_n_envlope','y_p_envlope','y_n_envlope',...
%         'z_p_envlope','z_n_envlope','vx_p_envlope','vx_n_envlope',...
%         'vy_p_envlope','vy_n_envlope','vz_p_envlope','vz_n_envlope')
%     
    
    
    
    
    
    
    figure(1)
    plot(t, x_hat(:,1),'b-')
    grid on
    hold on
    plot(t, x_true,'r-')
    %title('X')
     xlabel('$t_{k}$ [s]','Interpreter','latex','Fontsize',16)
     ylabel('$x_{k}$ [m]','Interpreter','latex','Fontsize',16)
     legend({'$Estimated~~x$','$Ground~~truth~~x$'},'Interpreter','latex','Fontsize',16);
    title({'$Estimated~~results~~for~~position~~x$'},'Interpreter','latex','Fontsize',16);
    hold off
    
    figure(2)
    plot(t, x_hat(:,2),'b-')
    grid on
    hold on
    plot(t, y_true,'r-')
    xlabel('$t_{k}$ [s]','Interpreter','latex','Fontsize',16)
    ylabel('$y_{k}$ [m]','Interpreter','latex','Fontsize',16)
    legend({'$Estimated~~y$','$Ground~~truth~~y$'},'Interpreter','latex','Fontsize',16);
    title({'$Estimated~~results~~for~~position~~y$'},'Interpreter','latex','Fontsize',16);
    hold off
    
    figure(3)
    plot(t, x_hat(:,3),'b-')
    grid on
    hold on
    plot(t, z_true,'r-')
    xlabel('$t_{k}$ [s]','Interpreter','latex','Fontsize',16)
    ylabel('$z_{k}$ [m]','Interpreter','latex','Fontsize',16)
    legend({'$Estimated~~z$','$Ground~~truth~~z$'},'Interpreter','latex','Fontsize',16);
    title({'$Estimated~~results~~for~~position~~z$'},'Interpreter','latex','Fontsize',16);
    hold off
    
    figure(4)
    plot(t, x_hat(:,4),'b-')
    grid on
    hold on
    plot(t, Vx_true,'r-')
    xlabel('$t_{k}$ [s]','Interpreter','latex','Fontsize',16)
    ylabel('$V_{xk}$ [m]','Interpreter','latex','Fontsize',16)
    legend({'$Estimated~~V_x$','$Ground~~truth~~V_x$'},'Interpreter','latex','Fontsize',16);
    title({'$Estimated~~results~~for~~position~~V_x$'},'Interpreter','latex','Fontsize',16);
    hold off
    
    figure(5)
    plot(t, x_hat(:,5),'b-')
    grid on
    hold on
    plot(t, Vy_true,'r-')
    xlabel('$t_{k}$ [s]','Interpreter','latex','Fontsize',16)
    ylabel('$V_{yk}$ [m]','Interpreter','latex','Fontsize',16)
    legend({'$Estimated~~V_y$','$Ground~~truth~~V_y$'},'Interpreter','latex','Fontsize',16);
    title({'$Estimated~~results~~for~~position~~V_y$'},'Interpreter','latex','Fontsize',16);
    hold off
    
    figure(6)
    plot(t, x_hat(:,6),'b-')
    grid on
    hold on
    plot(t, Vz_true,'r-')
    xlabel('$t_{k}$ [s]','Interpreter','latex','Fontsize',16)
    ylabel('$V_{zk}$ [m]','Interpreter','latex','Fontsize',16)
    legend({'$Estimated~~V_z$','$Ground~~truth~~V_z$'},'Interpreter','latex','Fontsize',16);
    title({'$Estimated~~results~~for~~position~~V_z$'},'Interpreter','latex','Fontsize',16);
    hold off

    t=time;
    
    figure(7)
    plot3(x_hat(:,1),x_hat(:,2),x_hat(:,3))
    grid on
    plot3(X_true,Y_true,Z_true)
    grid on
    hold on
    plot3(x_hat(:,1),x_hat(:,2),x_hat(:,3))
    title({'Estimated trajectory of crazyflie'},'Interpreter','latex','Fontsize',16);
    xlabel('$x$ [m]','Interpreter','latex','Fontsize',16);
    ylabel('$y$ [m]','Interpreter','latex','Fontsize',16);
    zlabel('$z$ [m]','Interpreter','latex','Fontsize',16);
    xlim([-1.5,1.5])
    ylim([-1.5,1.5])
    zlim([0,1.85])
    
%%  X error
    figure
    plot(t, error_x,'b-')
    title({'$Error~~x$'},'Interpreter','latex','Fontsize',16);
%     title(['\bf \fontsize{15} r_{max} = ', num2str(r_max)])
    xlabel('$t_{k}$ [s]','Interpreter','latex','Fontsize',16)
    ylabel('$\hat{x}_{k} - x_{k}$ [m]','Interpreter','latex','Fontsize',16)
    err_x_min = min(error_x);
    err_x_max = max(error_x);
%    ylim([err_x_min-epsilon err_x_max+epsilon])
    hold on    
    plot(t, x_p_envlope, 'r')
    plot(t, x_n_envlope, 'r')
    legend({'$x~error$','$\pm3\delta$'},'Interpreter','latex','Fontsize',16);
    grid on
    hold off
%% Y error     
    figure
    plot(t, error_y,'b-')
    title({'$Error~~y$'},'Interpreter','latex','Fontsize',16);
%    title(['\bf \fontsize{15} r_{max} = ', num2str(r_max)])
    xlabel('$t_{k} [s]$','Interpreter','latex','Fontsize',16)
    ylabel('$\hat{y}_{k} - y_{k}$ [m]','Interpreter','latex','Fontsize',16)
    err_y_min = min(error_y);
    err_y_max = max(error_y);
%    ylim([err_y_min-epsilon err_y_max+epsilon])
    hold on
    plot(t, y_p_envlope, 'r')
    plot(t, y_n_envlope, 'r')
    legend({'$y~error$','$\pm3\delta$'},'Interpreter','latex','Fontsize',16);
    grid on
    hold off
%% Z error     
    figure
    plot(t, error_z,'b-')
    title({'$Error~~z$'},'Interpreter','latex','Fontsize',16);
%    title(['\bf \fontsize{15} r_{max} = ', num2str(r_max)])
    xlabel('$t_{k}$ [s]','Interpreter','latex','Fontsize',16)
    ylabel('$\hat{z}_{k} - z_{k}$ [m]','Interpreter','latex','Fontsize',16)
    err_z_min = min(error_z);
    err_z_max = max(error_z);
%    ylim([err_z_min-epsilon err_z_max+epsilon])
    hold on
    plot(t, z_p_envlope, 'r')
    plot(t, z_n_envlope, 'r')
    legend({'$z~error$','$\pm3\delta$'},'Interpreter','latex','Fontsize',16);
    grid on
    hold off
    
%%  Vx  error
    figure
    plot(t, error_vx,'b-')
    title({'$Error~~v_{x}$'},'Interpreter','latex','Fontsize',16);
%     title(['\bf \fontsize{15} r_{max} = ', num2str(r_max)])
    xlabel('$t_{k} [s]$','Interpreter','latex','Fontsize',16)
    ylabel('$\hat{v}_{xk} - v_{xk}$ [m/s]','Interpreter','latex','Fontsize',16)
    err_vx_min = min(error_vx);
    err_vx_max = max(error_vx);
%    ylim([err_x_min-epsilon err_x_max+epsilon])
    hold on    
    plot(t, vx_p_envlope, 'r')
    plot(t, vx_n_envlope, 'r')
    legend({'$v_{x}~error$','$\pm3\delta$'},'Interpreter','latex','Fontsize',16);
    grid on
    hold off
%%  Vy error     
    figure
    plot(t, error_vy,'b-')
    title({'$Error~~v_{y}$'},'Interpreter','latex','Fontsize',16);
%    title(['\bf \fontsize{15} r_{max} = ', num2str(r_max)])
    xlabel('$t_{k}$ [s]','Interpreter','latex','Fontsize',16)
    ylabel('$\hat{v}_{yk} - v_{yk}$ [m/s]','Interpreter','latex','Fontsize',16)
    err_vy_min = min(error_vy);
    err_vy_max = max(error_vy);
%    ylim([err_y_min-epsilon err_y_max+epsilon])
    hold on
    plot(t, vy_p_envlope, 'r')
    plot(t, vy_n_envlope, 'r')
    legend({'$v_{y}~error$','$\pm3\delta$'},'Interpreter','latex','Fontsize',16);
    grid on
    hold off
%%  Vz error     
    figure
    plot(t, error_vz,'b-')
    title({'$Error~~v_{z}$'},'Interpreter','latex','Fontsize',16);
%    title(['\bf \fontsize{15} r_{max} = ', num2str(r_max)])
    xlabel('$t_{k}$ [s]','Interpreter','latex','Fontsize',16)
    ylabel('$\hat{v}_{zk} - v_{zk}$ [m/s]','Interpreter','latex','Fontsize',16)
    err_vz_min = min(error_vz);
    err_vz_max = max(error_vz);
%    ylim([err_z_min-epsilon err_z_max+epsilon])
    hold on
    plot(t, vz_p_envlope, 'r')
    plot(t, vz_n_envlope, 'r')
    legend({'$v_{z}~error$','$\pm3\delta$'},'Interpreter','latex','Fontsize',16);
    grid on
    hold off

end

%% ANIMATION

nframes=50;
Frames=moviein(nframes);
figure
axis tight manual
ax = gca;
ax.NextPlot = 'replaceChildren';
vidObj = VideoWriter('Crazyflie.avi','Motion JPEG AVI');
vidObj.FrameRate = 10;
vidObj.Quality = 100;
open(vidObj);
    plot3(x_true,y_true,z_true,'.b', 'markers',3);
    grid on;
    hold on;
    
for k = 1:length(time)
%jtrajectory3(x_hat(600,1),x_hat(600,2),x_hat(600,3),pitch(600),roll(600),yaw(600),1,floor(length(t)/15),'gripen', [22 34]);
    plot3(x_hat(k,1),x_hat(k,2),x_hat(k,3),'.r', 'markers',3);
%    title({'The animation of EKF estimation for crazyflie'},'Interpreter','latex','Fontsize',16);
    xlabel('$x$ [m]','Interpreter','latex','Fontsize',16);
    ylabel('$y$ [m]','Interpreter','latex','Fontsize',16);
    zlabel('$z$ [m]','Interpreter','latex','Fontsize',16);
    xlim([-1.5,1.5])
    ylim([-1.5,1.5])
    zlim([0,1.85])
    drawnow limitrate
   % hold off;  
    %% make the movie
    Fig(k) = getframe;
    currFrame = getframe(gcf);
    writeVideo(vidObj,currFrame);
end
close(vidObj);


end