%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run_KF_GPS_IMU
%
% Applies a Kalman filter to samples from cellphone data
% Two seperate update steps depending. 1 for GPS and 1 for IMU.
% Data was recorded with Sensor Play (Iphone) and AndroSensor (Android)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear all; clf(gcf()); clc;

% Editable filenames %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dataFile = strcat('mlt-20180307-172723-213_Ammar.csv');
% dataFile = strcat('mlt-20180307-173146-408_Ammar.csv');
dataFile = strcat('mlt-20180315-172636-9s_stationary.csv');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load(dataFile);    % Contains the noisy samples of vertical velocity
% data = readDataCSV_SP(dataFile);

%set Spherical Distance methods
% sphDist = pythagoreanLatLonDistance
% https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters
% Now apply the Kalman filter
% state is [x,x_d, x_dd, y, y_d, y_dd, z, z_d, z_dd, yaw, yaw_d, pitch,
% pitch_d, roll, roll_d]'
N = 15; %dimension of states
M_GPS = 3; %dimension of measurements
M_IMU = 6; %dimension of measurements
total_meas = size(data.IMU_data,1); % total measurements in file
% State Transition Matrix
% depends on dt so just call it each iteration

% Q = state transition covariance
Q = 0.5*diag(ones(N,1));

% H = state to measurement Matrix
H_IMU = KF_GPS_IMU.IMUstateMeasurementTransition();
H_GPS = KF_GPS_IMU.GPSstateMeasurementTransition();

% R =  noise covariance
R_GPS = 0.01*diag(ones(M_GPS,1));
R_IMU = 1*diag(ones(M_IMU,1));

KF = KF_GPS_IMU(Q, H_GPS, H_IMU, R_GPS, R_IMU);
IMU = KF_GPS_IMU(Q, H_GPS, H_IMU, R_GPS, R_IMU);
GPS = KF_GPS_IMU(Q, H_GPS, H_IMU, R_GPS, R_IMU);
% x_0 = [data.GPS_data(1,:)';zeros(N-3,1)];
x_0 = KF.initialPos(data.GPS_data(1,:));
P_0 = zeros(N,N);
KF = KF.setInitialValues(x_0, P_0);
IMU = IMU.setInitialValues(x_0, P_0);
GPS = GPS.setInitialValues(x_0, P_0);

filtered_est = zeros(total_meas, N);

i_gps = 1;
gps_update_idx = [];
% for i_imu =1:total_meas-1
for i_imu =1:1001
%     % generate state transition based on elapsed time
    dt = data.IMU_delta_time(i_imu);
    KF.Phi = KF.stateTransition(dt); % is this the same?
    IMU.Phi = IMU.stateTransition(dt);
    GPS.Phi = GPS.stateTransition(dt);
    % check GPS time if it is less than IMU time, then get that value
    
    IMU_sample = data.IMU_data(i_imu,:)';
    if data.GPS_time(i_gps) <  data.IMU_time(i_imu)
        [KF, x_kalman, cov, Kalman_gain] = KF.updateGPS(data.GPS_data(i_gps,:)');
        [GPS, x_GPS, ~, ~] = GPS.updateGPS(data.GPS_data(i_gps,:)');
        i_gps = i_gps + 1;
        filtered_est_gps(i_gps,:) = x_GPS';
        gps_update_idx = [gps_update_idx; i_imu];
        fprintf('%0.f GPS Point, Y Pos: %0.5f\n', i_gps, x_kalman(4));
        fprintf('Y Difference: %f\n', x_kalman(4) - filtered_est(i_imu-1, 4));
%         disp('Updating with GPS Location')
%         break
    else
        %move forward with IMU data
        [KF, x_kalman, cov, Kalman_gain] = KF.updateIMU(IMU_sample, data.phoneOrientation(i_imu,:));
%         fprintf('%0.f IMU Point, Y Pos: %0.5f\n', i_imu, x_kalman(4));
    end
    
    filtered_est(i_imu,:) = x_kalman';
    
    [IMU, x_dr, ~, ~] = IMU.updateIMU(IMU_sample, data.phoneOrientation(i_imu,:));
    filtered_est_dr(i_imu,:) = x_dr';
    
end

% Create plots of the output trajectory
idx_plot = filtered_est(:,1) ~= 0;
% scatter3(filtered_est(idx_plot,1), filtered_est(idx_plot,4),filtered_est(idx_plot,7))
hold on
scatter3(filtered_est_dr(idx_plot,1), filtered_est_dr(idx_plot,4),filtered_est_dr(idx_plot,7))
scatter3(filtered_est_gps(2:end,1), filtered_est_gps(2:end,4), filtered_est_gps(2:end,7), 'gx')
% scatter3(filtered_est(gps_update_idx,1), filtered_est(gps_update_idx, 4), filtered_est(gps_update_idx, 7), 'gx')
% data.plotGPSdata();
hold off
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
