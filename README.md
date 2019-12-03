## State Estimation and Bias Correction for a nano quadrotor
This is the final course project of AER 1513 State Estimation for Aerospace Vehicles, University of Toronto.

An extended Kalman Filter is built to estimate the fullstate of a nano quadrotor (Crazyflie). The fullstate includes
position, velocity and orientation. By fusing the onboard IMU and flowdeck sensor data, the estimator is shown to 
provide good estimation results in position z and velocity states comparing with groundtruth vicon data. For position
 x and y, estimated results have a little shift due to system observability issue.

We add the bias of IMU sensor into state vector and provide the bias correction through the estimation process. It is 
shown that bias correction can provide decent improvements in all  position and velocity states. 

The estimator is implemented offline based on the real dataset from Crazyflie platform. We logged data back when 
the nano quadrotor was doing a circle and built our real dataset. All the codes, including data process, EKF without bias 
correction and EKF with bias correction, are provided. The final report is in IROS template. 

The Extended Kalman Filter codes with and without bias compensation are contained in folder "Extended Kalman Filter". The main code is Crazyflie_EKF.m. For code testing, please run "Crazyflie_EKF(1,1)" in matlab terminal. Notice, the Aerospace Toolbox from Matlab is needed for the quaternion calculation.


