%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run_KF_GPS_IMU_SensorLoop
%
% Applies a Kalman filter to samples from cellphone data
% Two seperate update steps depending. 1 for GPS and 1 for IMU.
% Data was recorded with Sensor Play (Iphone) and AndroSensor (Android)
% Modelled so that the loop runs and only when the enough time has elapsed
% is another measurement provided
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear all; clf(gcf()); clc;

% Editable filenames %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dataFile = strcat('mlt-20180307-172723-213_Ammar.csv'); %North bound
% dataFile = strcat('mlt-20180307-173146-408_Ammar.csv'); % South Bound
% dataFile = strcat('mlt_20180307_172737_182.csv');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load(dataFile);    % Contains the noisy samples of vertical velocity
% data = readDataCSV_SP(dataFile);
data = data.restart();
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
loop_flag = true;
t_loop = data.IMU_time(1); dt_loop = 1e-3;
dt_imu = 0;
dt_gps = 0;
while loop_flag
    
    [data, IMU_meas, t_loop, dt_imu] = data.querryIMU(t_loop);
    [data, GPS_meas, t_loop, dt_gps] = data.querryGPS(t_loop);
    
    % generate state transition based on elapsed time
    if dt_imu == 0 && dt_gps == 0%just don't do any estimation between IMU readings
        t_loop = t_loop + dt_loop;
        continue
    end
    dt = max(dt_imu, dt_loop); % usually this would be kept track of by the loop
    
%     if ~isnan(IMU_meas)
%         disp(IMU_meas)
%     end
%     if ~isnan(GPS_meas)
%         disp(GPS_meas)
%     end
    
    GPS.Phi = GPS.stateTransition(dt);
    GPS_t.Phi = GPS_t.stateTransition(dt);
    
    if ~isnan(GPS_meas)
        i_gps = i_gps + 1;
        KF.Phi = KF.stateTransition(dt);
        [KF, ~, ~, ~] = KF.updateGPS(GPS_meas');
        KF = KF.saveData(t_loop);
        fprintf('%0.f GPS Point, Y Pos: %0.5f\n', i_gps, KF.post.x(4));
        GPS.Phi = GPS.stateTransition(dt);
        [GPS, ~, ~, ~] = GPS.updateGPS(GPS_meas');
        GPS = GPS.saveData(t_loop);
        [GPS_t, ~, ~, ~] = GPS_t.updateGPS(GPS_meas');
        GPS_t = GPS_t.saveData(t_loop);
    elseif ~isnan(IMU_meas)
        KF.Phi = KF.stateTransition(dt);
        [KF, ~, ~, ~] = KF.updateIMU(IMU_meas(1:6)', IMU_meas(7:9));
        KF = KF.saveData(t_loop);
        [GPS_t, ~, ~, ~] = GPS_t.updateGPS();
        GPS_t = GPS_t.saveData(t_loop);
        IMU.Phi = IMU.stateTransition(dt);
        [IMU, ~, ~, ~] = IMU.updateIMU(IMU_meas(1:6)', IMU_meas(7:9));
        IMU = IMU.saveData(t_loop);
    end
    % check GPS time if it is less than IMU time, then get that value

%     if i_gps < gps_total_meas && data.GPS_time(i_gps) <  data.IMU_time(i_imu)
%         [KF, ~, ~, ~] = KF.updateGPS(data.GPS_data(i_gps,:)');
%         [GPS, ~, ~, ~] = GPS.updateGPS(data.GPS_data(i_gps,:)');
%         i_gps = min(i_gps + 1, gps_total_meas);
%         GPS = GPS.saveData();
%         gps_update_idx = [gps_update_idx; i_imu];
%         fprintf('%0.f GPS Point, Y Pos: %0.5f\n', i_gps, KF.post.x(4));
%         [GPS_t, ~, ~, ~] = GPS_t.updateGPS(data.GPS_data(i_gps,:)');
%     else
%         %move forward with IMU data
%         [KF, x_kalman, cov, Kalman_gain] = KF.updateIMU(IMU_sample, data.phoneOrientation(i_imu,:));
%         [GPS_t, ~, ~, ~] = GPS_t.updateGPS();
%     end
%     GPS_t = GPS_t.saveData();
%     KF = KF.saveData();
%     [IMU, ~, ~, ~] = IMU.updateIMU(IMU_sample, data.phoneOrientation(i_imu,:));
%     IMU = IMU.saveData();
    t_loop = t_loop + dt_loop;
    if data.trialEndCheck()
        loop_flag = false;
        disp('Trial Ended')
    end
end


pts = 10;
% GPS=GPS.zeroStart(); KF=KF.zeroStart(); GPS_t=GPS_t.zeroStart();
% data = data.zeroStart();
t_step_IMU = data.IMU_time(1:pts:length(IMU.x_all),1)-data.IMU_time(1);
t_step_GPS = data.GPS_time(1:end-1,1)-data.GPS_time(1);
% 
% 
% % KF over GPS
figure('Name', 'KF over GPS', 'units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1)
plot(GPS.t_all-GPS.t_all(1), GPS.x_all, 'rx-', t_step_GPS, data.GPS_data(1:end-1,1), 'g^')
xlabel('Time(s)'); ylabel('North(m)');
legend('KF', 'GPS')
title('Kalman Filter over GPS');
% East
subplot(3,1,2)
plot(GPS.t_all-GPS.t_all(1), GPS.y_all, 'rx-', t_step_GPS, data.GPS_data(1:end-1,2), 'g^')
xlabel('Time(s)'); ylabel('East(m)');
% Down
subplot(3,1,3)
plot(GPS.t_all-GPS.t_all(1), GPS.z_all, 'rx-', t_step_GPS, data.GPS_data(1:end-1,3), 'g^')
xlabel('Time(s)'); ylabel('Down(m)');
% 
% % KF over GPS and IMU data
figure('Name', 'KF over GPS and IMU', 'units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1)
plot(KF.t_all(1:pts:end)-KF.t_all(1), KF.x_all(1:pts:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,1), 'g^')
ylim([-20,500]);
xlabel('Time(s)'); ylabel('North(m)');
legend('KF', 'GPS')
title('Kalman Filter over GPS and IMU Readings');
% East
subplot(3,1,2)
plot(KF.t_all(1:pts:end)-KF.t_all(1), KF.y_all(1:pts:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,2), 'g^')
xlabel('Time(s)'); ylabel('East(m)');
ylim([-20,500]);
% Down
subplot(3,1,3)
plot(KF.t_all(1:pts:end)-KF.t_all(1), KF.z_all(1:pts:end), 'rx-', t_step_GPS, data.GPS_data(1:end-1,3), 'g^')
xlabel('Time(s)'); ylabel('Down(m)');
ylim([-20,500]);


% Zoom: KF over GPS
figure('Name', 'KF over GPS', 'units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1)
plot(GPS.t_all-GPS.t_all(1), GPS.x_all, 'rx-', t_step_GPS, data.GPS_data(1:end-1,1), 'g^')
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
% 
% % 3D plot of points