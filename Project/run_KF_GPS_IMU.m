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
dataFile = strcat('mlt-20180307-172723-213_Ammar.csv'); %North bound
% dataFile = strcat('mlt-20180307-173146-408_Ammar.csv'); % South Bound
% dataFile = strcat('mlt_20180307_172737_182.csv');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load(dataFile);    % Contains the noisy samples of vertical velocity
data = readDataCSV_SP(dataFile);
data = data.zeroStart();
% Now apply the Kalman filter
% state is [x,x_d, x_dd, y, y_d, y_dd, z, z_d, z_dd, roll, roll_d, pitch,
% pitch_d, yaw, yaw_d]'
N = 15; %dimension of states
M_GPS = 3; %dimension of measurements
M_IMU = 6; %dimension of measurements
total_meas = size(data.IMU_data,1); % total measurements in file
gps_total_meas = size(data.GPS_time,1);

% State Transition Matrix
% depends on dt so just call it each iteration

% Q = state transition covariance
% decreasing Q makes it follow slower
Q = 5*diag(ones(N,1));

% H = state to measurement Matrix
H_IMU = KF_GPS_IMU.IMUstateMeasurementTransition();
H_GPS = KF_GPS_IMU.GPSstateMeasurementTransition();

% R =  noise covariance
IMU_noise = 100; GYRO_noise = 1; GPS_noise = 10;
% increasing GPS noise makes it follow worse
% decreasing IMU noise makes it follow about the same
R_GPS = GPS_noise*diag(ones(M_GPS,1));
R_IMU = diag([IMU_noise; IMU_noise; IMU_noise;GYRO_noise; GYRO_noise; GYRO_noise]);

KF = KF_GPS_IMU(Q, H_GPS, H_IMU, R_GPS, R_IMU);
IMU = KF_GPS_IMU(Q, H_GPS, H_IMU, R_GPS, R_IMU);
GPS = KF_GPS_IMU(Q, H_GPS, H_IMU, R_GPS, R_IMU);
GPS_t = KF_GPS_IMU(Q, H_GPS, H_IMU, R_GPS, R_IMU);

x_0 = KF.initialPos(data.GPS_data(2,:));
P_0 = zeros(N,N);
KF = KF.setInitialValues(x_0, P_0);
IMU = IMU.setInitialValues(x_0, P_0);
GPS = GPS.setInitialValues(x_0, P_0);
GPS_t = GPS_t.setInitialValues(x_0, P_0);

filtered_est = zeros(total_meas, N);

i_gps = 1;
gps_update_idx = [];
for i_imu =1:total_meas-1
% for i_imu =1:1001
%     % generate state transition based on elapsed time
    dt = data.IMU_delta_time(i_imu);
    KF.Phi = KF.stateTransition(dt);
    IMU.Phi = IMU.stateTransition(dt);
    GPS.Phi = GPS.stateTransition(dt);
    GPS_t.Phi = GPS_t.stateTransition(dt);
    
    % check GPS time if it is less than IMU time, then get that value
    IMU_sample = data.IMU_data(i_imu,:)';
    if i_gps < gps_total_meas && data.GPS_time(i_gps) <  data.IMU_time(i_imu)
        [KF, ~, ~, ~] = KF.updateGPS(data.GPS_data(i_gps,:)');
        [GPS, ~, ~, ~] = GPS.updateGPS(data.GPS_data(i_gps,:)');
        i_gps = min(i_gps + 1, gps_total_meas);
        GPS = GPS.saveData();
        gps_update_idx = [gps_update_idx; i_imu];
        fprintf('%0.f GPS Point, Y Pos: %0.5f\n', i_gps, KF.post.x(4));
        [GPS_t, ~, ~, ~] = GPS_t.updateGPS(data.GPS_data(i_gps,:)');
    else
        %move forward with IMU data
        [KF, x_kalman, cov, Kalman_gain] = KF.updateIMU(IMU_sample, data.phoneOrientation(i_imu,:));
        [GPS_t, ~, ~, ~] = GPS_t.updateGPS();
    end
    GPS_t = GPS_t.saveData();
    KF = KF.saveData();
    [IMU, ~, ~, ~] = IMU.updateIMU(IMU_sample, data.phoneOrientation(i_imu,:));
    IMU = IMU.saveData();
    
end

% Create plots of the output trajectory
% idx_plot = filtered_est(:,1) ~= 0;
% scatter3(filtered_est(idx_plot,1), filtered_est(idx_plot,4),filtered_est(idx_plot,7))
% hold on
% scatter3(filtered_est_dr(idx_plot,1), filtered_est_dr(idx_plot,4),filtered_est_dr(idx_plot,7))
% scatter3(filtered_est_gps(2:end,1), filtered_est_gps(2:end,4), filtered_est_gps(2:end,7), 'gx')
% scatter3(filtered_est(gps_update_idx,1), filtered_est(gps_update_idx, 4), filtered_est(gps_update_idx, 7), 'gx')
% data.plotGPSdata();
% hold off
% xlabel('x')
% ylabel('y')
% zlabel('z')
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


pts = 10;
% GPS=GPS.zeroStart(); KF=KF.zeroStart(); GPS_t=GPS_t.zeroStart();
% data = data.zeroStart();
t_step_IMU = data.IMU_time(1:pts:length(IMU.x_all),1)-data.IMU_time(1);
t_step_GPS = data.GPS_time(1:end-1,1)-data.GPS_time(1);


% KF over GPS
figure('Name', 'KF over GPS', 'units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1)
plot(t_step_GPS, GPS.x_all(1:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,1), 'g^')
xlabel('Time(s)'); ylabel('North(m)');
legend('KF', 'GPS')
title('Kalman Filter over GPS');
% East
subplot(3,1,2)
plot(t_step_GPS, GPS.y_all(1:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,2), 'g^')
xlabel('Time(s)'); ylabel('East(m)');
% Down
subplot(3,1,3)
plot(t_step_GPS, GPS.z_all(1:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,3), 'g^')
xlabel('Time(s)'); ylabel('Down(m)');
saveas(gcf, 'KFOverGPS.png');

% KF over GPS and IMU data
figure('Name', 'KF over GPS and IMU', 'units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1)
plot(t_step_IMU, KF.x_all(1:pts:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,1), 'g^')
xlabel('Time(s)'); ylabel('North(m)');
legend('KF', 'GPS')
title('Kalman Filter over GPS and IMU Readings');
% East
subplot(3,1,2)
plot(t_step_IMU, KF.y_all(1:pts:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,2), 'g^')
xlabel('Time(s)'); ylabel('East(m)');
% Down
subplot(3,1,3)
plot(t_step_IMU, KF.z_all(1:pts:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,3), 'g^')
xlabel('Time(s)'); ylabel('Down(m)');
ylim([0,500]);
saveas(gcf, 'KFOverGPSIMU.png');


% Zoom: KF over GPS
figure('Name', 'KF over GPS', 'units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1)
plot(t_step_GPS, GPS.x_all(1:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,1), 'g^')
xlim([150,160]);
xlabel('Time(s)'); ylabel('North(m)');
legend('KF', 'GPS')
title('Zoom: Kalman Filter over GPS');
% East
subplot(3,1,2)
plot(t_step_GPS, GPS.y_all(1:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,2), 'g^')
xlim([150,160]);
xlabel('Time(s)'); ylabel('East(m)');
% Down
subplot(3,1,3)
plot(t_step_GPS, GPS.z_all(1:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,3), 'g^')
xlim([150,160]);
xlabel('Time(s)'); ylabel('Down(m)');
saveas(gcf, 'ZoomKFOverGPS.png');

% Zoom in on KF over GPS and IMU data
figure('Name', 'Zoom: KF over GPS and IMU', 'units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1)
plot(t_step_IMU, KF.x_all(1:pts:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,1), 'g^')
xlim([150,160]);
xlabel('Time(s)'); ylabel('North(m)');
legend('KF', 'GPS')
title('Zoom: Kalman Filter over GPS and IMU Readings');
% East
subplot(3,1,2)
plot(t_step_IMU, KF.y_all(1:pts:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,2), 'g^')
xlim([150,160]);
xlabel('Time(s)'); ylabel('East(m)');
% Down
subplot(3,1,3)
plot(t_step_IMU, KF.z_all(1:pts:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,3), 'g^')
xlim([150,160]);
xlabel('Time(s)'); ylabel('Down(m)');
saveas(gcf, 'ZoomKFOverGPSIMU.png');

% Zoom: KF over GPS when GPS still predicts at every time step
figure('Name', 'KF over GPS, Predict w/o Meas', 'units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1)
plot(t_step_IMU, GPS_t.x_all(1:pts:end), 'rx-', t_step_IMU, KF.x_all(1:pts:end), 'bo-', t_step_GPS, data.GPS_data(1:end-1,1), 'g^')
xlim([150,160]);
xlabel('Time(s)'); ylabel('North(m)');
legend('KF-GPS', 'KF-GPS/IMU', 'GPS')
title('Zoom: Kalman Filter over GPS, Predict w/o Meas');
% East
subplot(3,1,2)
plot(t_step_IMU, GPS_t.y_all(1:pts:end), 'rx-', t_step_IMU, KF.y_all(1:pts:end), 'bo-', t_step_GPS, data.GPS_data(1:end-1,2), 'g^')
xlim([150,160]);
xlabel('Time(s)'); ylabel('East(m)');
% Down
subplot(3,1,3)
plot(t_step_IMU, GPS_t.z_all(1:pts:end), 'rx-', t_step_IMU, KF.z_all(1:pts:end), 'bo-', t_step_GPS, data.GPS_data(1:end-1,3), 'g^')
xlim([150,160]);
xlabel('Time(s)'); ylabel('Down(m)');
saveas(gcf, 'ZoomKFOverGPS_w-oMeas.png');

% 3D plot of points