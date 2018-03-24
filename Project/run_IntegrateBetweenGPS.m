%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run_IntegrateBetweenGPS
%
% Simplest way to process GNSS data and IMU data
% Between GNSS samples, use the IMU to locate position
% Data was recorded with Sensor Play (Iphone) and AndroSensor (Android)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear all; clf(gcf()); clc;

% Editable filenames %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dataFile = strcat('mlt-20180307-172723-213_Ammar.csv');
% dataFile = strcat('mlt_20180307_172737_182.csv');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load(dataFile);    % Contains the noisy samples of vertical velocity
data = readDataCSV_SP(dataFile);
data = data.zeroStart();

% run forward through data
imu_total_meas = size(data.IMU_data,1); % total measurements in file
gps_total_meas = size(data.GPS_time,1);
% initial values
x0 = data.GPS_data(1,1);
y0 = data.GPS_data(1,2);
z0 = data.GPS_data(1,3);
x_d = 0; y_d = 0; z_d = 0;
x_dd = 0; y_dd = 0; z_dd = 0;


x_all = zeros(imu_total_meas-1,1);
y_all = zeros(imu_total_meas-1,1);
z_all = zeros(imu_total_meas-1,1);
dt_all = zeros(imu_total_meas-1,1);

IGPS = IntegrateBetweenGPS();
DR = IntegrateBetweenGPS();  % does not use updates from GPS

i_gps = 1;
gps_update_idx = [];
for i_imu =1:imu_total_meas-1
% for i_imu = 1:1000
    % check GPS time if it is less than IMU time, then get that value
    if i_gps < gps_total_meas && data.GPS_time(i_gps) <  data.IMU_time(i_imu)
%         disp('Updating with GPS Location')
        IGPS = IGPS.updateGPS(data.GPS_data(i_gps,:));
        if i_gps == 1 % get the first GPS reading then dead reckon
            DR = DR.updateGPS(data.GPS_data(i_gps,:));
        end
        gps_update_idx = [gps_update_idx; i_imu];
        i_gps = min(i_gps + 1, gps_total_meas);
    elseif i_gps > 1 % wait for first GPS reading
        %move forward with IMU data
%         disp('Using only IMU data')
        % need to rotate accelerations into current frame
        dt = data.IMU_delta_time(i_imu);
        dt_all(i_imu) = dt;
        IGPS = IGPS.updateIMU(data.IMU_data(i_imu,:), data.phoneOrientation, dt);
        DR = DR.updateIMU(data.IMU_data(i_imu,:), data.phoneOrientation, dt);
    elseif i_gps == 1
        continue
    end
    
    IGPS = IGPS.saveData();
    DR = DR.saveData();
    
end

% Create plots of the output trajectory
% axesm utm
% h = getm(gca);
% h.zone
% setm(gca,'zone','10n')
% setm(gca, 'maplatlimit',[min(x_all), max(x_all)])
% setm(gca, 'maplonlimit',[min(y_all), max(y_all)])
% h = getm(gca);
% setm(gca,'grid','on','meridianlabel','on','parallellabel','on')
% plotm(x_all, y_all, 'rx')
% scatter3(IGPS.x_all(1:end-1), IGPS.y_all(1:end-1), IGPS.z_all(1:end-1), 'bo')
% hold on;
% scatter3(IGPS.x_all(gps_update_idx), IGPS.y_all(gps_update_idx), IGPS.z_all(gps_update_idx), 'g*')
% figure()
% scatter3(DR.x_all(1:end-1),DR.y_all(1:end-1),DR.z_all(1:end-1), 'rx')
% figure()
% hold on
% plot(IGPS.x_all(1:end-1), IGPS.y_all(1:end-1), 'bx')
% plot(IGPS.x_all(gps_update_idx), IGPS.y_all(gps_update_idx), 'ro')
% plot(DR.x_all(1:end-1),DR.y_all(1:end-1), 'g*')
% hold off
% figure()
% plot(x_all, y_all, 'rx')
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

% %  % % plot motion in each dimension % %  % % 
pts = 10;
t_steps_IMU = data.IMU_time(1:pts:end-pts,1)-data.IMU_time(1);
t_step_GPS = data.GPS_time(1:end-1,1)-data.GPS_time(1);
% Just the Path
figure('Name', 'Integrate - Filter Path Only', 'units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1)
plot(t_steps_IMU, IGPS.x_all(1:pts:end), 'r',t_step_GPS , data.GPS_data(1:end-1,1), 'g^')
xlabel('Time(s)'); ylabel('North(m)');
legend('Integrated', 'GPS')
title('Integrating Between GPS Readings');
% East
subplot(3,1,2)
plot(t_steps_IMU, IGPS.y_all(1:pts:end-1), 'r',t_step_GPS , data.GPS_data(1:end-1,2), 'g^')
xlabel('Time(s)'); ylabel('East(m)');
% Down
subplot(3,1,3)
plot(t_steps_IMU, IGPS.z_all(1:pts:end-1), 'r',t_step_GPS , data.GPS_data(1:end-1,3), 'g^')
xlabel('Time(s)'); ylabel('Down(m)');
saveas(gcf, 'IntegratePath.png');

% Dead Reckoning
figure('Name', 'Integrate - Compare to Dead Reckoning', 'units','normalized','outerposition',[0 0 1 1])
% North
subplot(3,1,1)
plot(t_steps_IMU, DR.x_all(1:pts:end-1), 'b', t_step_GPS , data.GPS_data(1:end-1,1), 'g^')
xlabel('Time(s)'); ylabel('North(m)');
legend('Dead Reckoning', 'GPS')
title('Dead Reckoning with IMU')
% East
subplot(3,1,2)
plot(t_steps_IMU, DR.y_all(1:pts:end-1), 'b', t_step_GPS , data.GPS_data(1:end-1,2), 'g^')
xlabel('Time(s)'); ylabel('East(m)');
% Down
subplot(3,1,3)
plot(t_steps_IMU, DR.z_all(1:pts:end-1), 'b', t_step_GPS , data.GPS_data(1:end-1,3), 'g^')
xlabel('Time(s)'); ylabel('Down(m)');
saveas(gcf, 'DeadReckoning.png');

