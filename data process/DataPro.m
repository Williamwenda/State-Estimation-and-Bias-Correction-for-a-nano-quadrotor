clc
close all
clear all
%%
FolderName = '/home/william/Desktop/Homework/State Estimation/SE-finalpro/State Estimation and Bisa Correction for a nano quadrotor/data process/rawdata/';
FileName   = 'ViconZ_V.mat';
ViconZV    = fullfile(FolderName, FileName);
FileName   = 'kalman_state.mat';
Kalman    = fullfile(FolderName, FileName);
FileName   = 'IMU_Vx.mat';
IMU    = fullfile(FolderName, FileName);
FileName   = 'flowdeck_Vy.mat';
Flowdeck    = fullfile(FolderName, FileName);
%% load data and draw the trajectory
load(ViconZV);
timeV=(time-time(1))';
load(Kalman);
timeK=(time-time(1))';
load(IMU);
timeI=(time-time(1))';
load(Flowdeck);
timeF=(time-time(1))';
%% Show figures
showOD=false;
showTra=false;
showAli=false;
showZ=false;
showVar=false;
showPN=false;
%% Here is the orignal data
while showOD
    Vx1=flowdeck_Vy(:,2);
    Vx2=ViconZ_V(:,2);
    Vx1=Vx1(1:length(Vx2));
    figure;
    plot(timeV,Vx1,timeV,Vx2);
    grid on
    legend('Measurement Vx','ground truth Vx');
    title('Before data alignment');
    break
end
%%
IMU_Vx = align(IMU_Vx,timeI,flowdeck_Vy,timeF);
ViconZ_V = align(ViconZ_V,timeV,flowdeck_Vy,timeF);
kalman_state=align(kalman_state,timeK,flowdeck_Vy,timeF);
%% Here need to double check
%% Translate from 1662 to 1661
D=length(IMU_Vx);
flowdeck_Vy = flowdeck_Vy(1:D,:);
time=timeF(1:D);
clear timeI timeK timeV timeF
%% true position from VICON
x_true=IMU_Vx(:,4);
y_true=flowdeck_Vy(:,4);
z_true=ViconZ_V(:,1);
state_true=[x_true,y_true,z_true];
%% Show trajectory
while showTra
    figure;
    plot3(x_true,y_true,z_true);
    grid on;
    axis([-1.5 1.5 -1.5 1.5 0 1.75]);
    break
end

%% Set the measurement data
Vx_f=flowdeck_Vy(:,2);
Vy_f=flowdeck_Vy(:,3);
%% Z range measurement from flow deck
rangeZ=flowdeck_Vy(:,1);
rangeZ=rangeZ(1:D)/1000;  %% translate into meters
%% show Z range data
while showZ
    figure
    plot(time,rangeZ,time,z_true)  %% Before processing
    grid on
    legend('rangeZ','ViconZ');
    title('Before filtering');
    rangeZ(find(rangeZ>5))=1.5;
    figure
    plot(time,rangeZ,time,z_true)  %% After processing, rangeZ*10 just for show
    grid on
    legend('rangeZ','ViconZ');
    title('After filtering');
    break
end
rangeZ(find(rangeZ>5))=1.5;

%% Add random noise to the measurements
m=0;
stdv=0.03;
Vx_f=Vx_f+stdv*randn(size(Vx_f))+m;
Vy_f=Vy_f+stdv*randn(size(Vy_f))+m;
%%
Meas=[rangeZ,Vx_f, Vy_f];
%% show aligned data
while showAli
    figure;
    plot(time,Vx_f,time,ViconZ_V(:,2));
    grid on
    legend('Measurement Vx','ground truth Vx');
    title('After data alignment');
    figure;
    plot(time,Vy_f,time,ViconZ_V(:,3));
    grid on
    legend('Measurement Vy','ground truth Vy');
    title('After data alignment');
    break
end
%% Rotation matrix R
R=zeros(3,3,D);
q0=kalman_state(:,1);
qv=kalman_state(:,2:4);
%% Rotate the Velocity from body frame to global frame
for i=1:D
    qx=[0       -qv(i,3)   qv(i,2);
        qv(i,3)     0     -qv(i,1);
       -qv(i,2)  qv(i,1)     0  ];
    R(:,:,i)=(2*q0(i)^2-1)*eye(3)+2*qv(i,:)*qv(i,:)'-2*q0(i)*qx;
    % here can translate from body frame to world frame
end

%% true velocity from VICON
Vx_true=ViconZ_V(:,2);
Vy_true=ViconZ_V(:,3);
Vz_true=ViconZ_V(:,4);
stateV_true=[Vx_true, Vy_true, Vz_true];

%% Calculate the variance of measurement data
Err_z = Meas(:,1)-z_true;
Mean_Ez=mean(Err_z);
Var_Ez=var(Err_z);

Err_Vx=Meas(:,2)-Vx_true;
Mean_Evx=mean(Err_Vx);
Var_Evx=var(Err_Vx);

Err_Vy=Meas(:,3)-Vy_true;
Mean_Evy=mean(Err_Vy);
Var_Evy=var(Err_Vy);
%% Calculate the time interval
for i=1:length(time)-1
    int(i)=time(i+1)-time(i);
end
%T=mean(1./int);    % unit is Hz
T=mean(int);        % unit is second

%% Show variance of measurement noise
while showVar
    figure
    h=histfit(Err_z);
    pd=fitdist(Err_z,'Normal');
    title('$Histogram~~of~~Err_z$','Interpreter','latex','Fontsize',16); 
    txt={['Mean:' num2str(Mean_Ez)] , ['Var:' num2str(Var_Ez)]};
    text(0.05,300,txt);

    figure
    h=histfit(Err_Vx);
    pd=fitdist(Err_Vx,'Normal');
    title('$Histogram~~of~~Err\_V_x$','Interpreter','latex','Fontsize',16);
    txt={['Mean:' num2str(Mean_Evx)] , ['Var:' num2str(Var_Evx)]};
    text(0.3,500,txt);

    figure
    h=histfit(Err_Vy);
    pd=fitdist(Err_Vy,'Normal');
    title('$Histogram~~of~~Err\_V_y$','Interpreter','latex','Fontsize',16);
    txt={['Mean:' num2str(Mean_Evy)] , ['Var:' num2str(Var_Evy)]};
    text(-0.4,400,txt);
    break
end
%% Compute the process noise
%  input is ax,ay,az
acc=[IMU_Vx(:,1), IMU_Vx(:,2), IMU_Vx(:,3)]';
%  time from 1 to 219, crazyflie doesn't move. The integration of acc
%  shoule be zero
%  acc_tem=acc(:, 1:219);
%  R_tem=R(:, :, 1:219);
gravity=9.80665;
Integ_p=zeros(3,219);
Integ_v=zeros(3,219);
for j=1:219 
    Integ_p(:,j)=0.5*T^2*(R(:,:,j)*acc(:,j)-[gravity;gravity;gravity]);
    Integ_v(:,j)=T*(R(:,:,j)*acc(:,j)-[gravity;gravity;gravity]);
end
P_mean=[mean(Integ_p(1,:)), mean(Integ_p(2,:)), mean(Integ_p(3,:))];
P_var=[var(Integ_p(1,:)), var(Integ_p(2,:)), var(Integ_p(3,:))];
V_mean=[mean(Integ_v(1,:)), mean(Integ_v(2,:)), mean(Integ_v(3,:))];
V_var=[var(Integ_v(1,:)), var(Integ_v(2,:)), var(Integ_v(3,:))];
while showPN
    figure
    h=histfit(Integ_p(1,:)');
    pd=fitdist(Integ_p(1,:)','Normal');
break
end

%% Variance
%  Both the variances of process and measurement noise are very small ...
R_var=diag([Var_Ez,Var_Evx,Var_Evy]);
Q_var=diag([P_var,V_var]);
%% Output the data 
%% build the dataset
%  time: time series,  state_true: x,y,z position from Vicon,  stateV_true: Vx,Vy,Vz velocity from Vicon
%  acc: input accelaration,  R: rotation matrix,  Meas: measurements from flowdeck
%  R_var: process noise variance,  Q_var: measurement noise variance,  gravity: gravity accelerate    
%  D: the dimension    T: time step
save('cleandata/dataset.mat','time','state_true','stateV_true','acc','R','Meas','R_var','Q_var','gravity','D','T');








