%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run_KF_6D_IMU
%
% Applies a Kalman filter to samples from cellphone data
% Data was recorded with Sensor Play (Iphone) and AndroSensor (Android)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear all; clf(gcf()); clc;

% Editable filenames %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dataFile = strcat('mlt-20180307-172723-213_Ammar.csv');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load(dataFile);    % Contains the noisy samples of vertical velocity
data = readDataCSV_SP(dataFile);

%set Spherical Distance methods
% sphDist = pythagoreanLatLonDistance
% https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters
% Now apply the Kalman filter
% state is [x,x_d, x_dd, y, y_d, y_dd, z, z_d, z_dd, yaw, yaw_d, pitch,
% pitch_d, roll, roll_d]'
N = 15; %dimension of states
M = 6; %dimension of measurements
total_meas = size(data.IMU_data,1); % total measurements in file
% State Transition Matrix
% depends on dt so just call it each iteration

% Q = state transition covariance
Q = 0.5*diag(ones(N,1));

% H = state to measurement Matrix
H = KF_6D_IMU.stateMeasurementTransition();

% R =  noise covariance
R = 0.1*diag(ones(M));

KF = KF_6D_IMU(Q, H, R);
x_0 = zeros(N,1);
P_0 = zeros(N,N);
KF = KF.setInitialValues(x_0, P_0);

filtered_est = zeros(total_meas, N);
for i =1:total_meas-1
    IMU_sample = data.IMU_data(i,:)';
    % generate state transition based on elapsed time
    dt = data.IMU_delta_time(i);
    KF.Phi = KF.stateTransition(dt);
    [KF, x_kalman, cov, Kalman_gain] = KF.update(IMU_sample);
    
    filtered_est(i,:) = x_kalman';
    
    % do we need to transfer the reference frame back to the original frame?
%     Kg(i,:) = Kalman_gain;
%     P_norm(i) = norm(cov);
end

% Create plots of the output trajectory
scatter3(filtered_est(:,1), filtered_est(:,4),filtered_est(:,7))
xlabel('x')
ylabel('y')
zlabel('z')
% hold on
% subplot(2,1,1)
% plot(tIndx,filtered_h_el,'r','LineWidth',2.5)
% title('Kalman Filter Results')
% xlabel('Time (sec)');
% ylabel('Altitude (m)');
% legend('Kalman filtered trajectory')
% subplot(2,1,2)
% plot(tIndx,filtered_vel,'g','LineWidth',2.5)
% xlabel('Time (sec)');
% ylabel('Velocity (m/s)');
% hold on
% plot(tIndx,vel_samples)
% legend('Kalman filtered','Noisy Velocity Samples')
