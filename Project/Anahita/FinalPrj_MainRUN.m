clc
clear all
close all
%-->> Initial considerations:
% Reference datum for convertion
ref_datum = 'WGS 84';

t0 = 0;
velocity0 = 0;
position0 = 0;
freq = 100;
g = 9.81;
%--------------------------------------------------------------------------
%-->> Reading Multi-sensor measuremednt from SensorPlay, iPhone6
filename = dir('*.csv');
[data_trajectory,header_trajectory] = xlsread(filename.name);
data_trajectory(1:2,:) = [];

t = data_trajectory(:,1); t = datenum(data_trajectory(:,1));
accel_x = data_trajectory(:,2)*g;
accel_y = data_trajectory(:,3)*g;
accel_z = data_trajectory(:,4)*g + g;
gyro_x = data_trajectory(:,5); % rad/sec
gyro_y = data_trajectory(:,6); % rad/sec
gyro_z = data_trajectory(:,7); % rad/sec
roll = rad2deg(data_trajectory(:,8)); % rad->deg
pitch = rad2deg(data_trajectory(:,9)); % rad->deg
yaw = rad2deg(data_trajectory(:,10)); % rad->deg
lat = data_trajectory(:,11); % deg 
lat0 = lat(1);
lon = data_trajectory(:,12); % deg
lon0 = lon(1);
speed = data_trajectory(:,13); % mph
treu_heading = data_trajectory(:,14);
alt = data_trajectory(:,15); alt = distdim(alt,'ft','m'); % ft -> m

% vel_N = data_trajectory(:,5); % m/s
% vel_E = data_trajectory(:,6); % m/s
% vel_D = data_trajectory(:,7); % m/s


%..........................................................................
lat0 = lat(1);
lon0 = lon(1);
alt0 = alt(1);
spheroid = referenceEllipsoid(ref_datum);
xyz_ecef = lla2ecef([lat,lon,alt]);
[N,E,D] = ecef2ned(xyz_ecef(:,1),xyz_ecef(:,2),xyz_ecef(:,3),lat0,lon0,alt0,spheroid);
% [N,E,D] = geodetic2ned(lat,lon,alt,lat0,lon0,alt0,spheroid);
%..........................................................................
[vel_N] = compute_velocity_NED(N);
[vel_E] = compute_velocity_NED(E);
[vel_D] = compute_velocity_NED(D);
%..........................................................................

% position0 = (lla2ecef([lat0,lon0,alt0]))';
velocity0 = [0 0 0]';
position0 = [0 0 0]';
% % % [DCM_earth_to_local] = earth2local_frame(lat0,lon0);
% % % DCM_local_to_earth = DCM_earth_to_local';
% % % [DCM_body_to_NED] = body2NED_frame(deg2rad(roll(1)),deg2rad(pitch(1)),deg2rad(yaw(1)));
% % % DCM_local_to_body = DCM_body_to_NED';
% % % position0 = DCM_local_to_earth * position0;
% % % position0 = DCM_local_to_body * position0;
% % % velocity0 = DCM_local_to_earth * velocity0;
% % % velocity0 = DCM_local_to_body * velocity0;

t0 = 0;
[vx_imu,posX_imu,~] = pos_vel_from_IMU(accel_x,t0,velocity0(1),position0(1),freq);
[vy_imu,posY_imu,~] = pos_vel_from_IMU(accel_y,t0,velocity0(2),position0(2),freq);
[vz_imu,posZ_imu,~] = pos_vel_from_IMU(accel_z,t0,velocity0(3),position0(3),freq);


for i=1:length(accel_x)
    prmtr_body = [posX_imu(i) posY_imu(i) posZ_imu(i)]';
    [DCM_body_to_NED] = body2NED_frame(deg2rad(roll(i)),deg2rad(pitch(i)),deg2rad(yaw(i)));
    prmtr_local = DCM_body_to_NED * prmtr_body;
    posX_imu_NED(i,1) = prmtr_local(1);
    posY_imu_NED(i,1) = prmtr_local(2);
    posZ_imu_NED(i,1) = prmtr_local(3);
    
    prmtr_body = [vx_imu(i) vy_imu(i) vz_imu(i)]';
    prmtr_local = DCM_body_to_NED * prmtr_body;
    vx_imu_NED(i,1) = prmtr_local(1);
    vy_imu_NED(i,1) = prmtr_local(2);
    vz_imu_NED(i,1) = prmtr_local(3);
end
clear prmtr_body prmtr_local
% figure;hold on;plot(t,vel_N,'b');plot(t,velN_imu,'r')
% figure;hold on;plot(t,vel_E,'b');plot(t,velE_imu,'r')
% figure;hold on;plot(t,vel_D,'b');plot(t,velD_imu,'r')
% figure;hold on;plot(t,vel_N,'b');plot(t,vx_imu,'r')



sigma_Q = 25;
sigma_R =  0.9;
delta_t = 1/freq;
for i =1:length(t)
    obs = [posX_imu_NED(i)-N(i);posY_imu_NED(i)-E(i);posZ_imu_NED(i)-D(i);vx_imu_NED(i)-vel_N(i);vy_imu_NED(i)-vel_E(i);vz_imu_NED(i)-vel_D(i)];
    [x_kalman, cov, Kalman_gain] = kalman_filter(obs,lat(i),lon(i),alt(i),delta_t,vel_N(i),vel_E(i),vel_D(i),roll(i),pitch(i),yaw(i),sigma_Q,sigma_R);
    
    filtered_x(i) = x_kalman(1);
    filtered_y(i) = x_kalman(2);
    filtered_z(i) = x_kalman(3);
    filtered_vel_x(i) = x_kalman(4);
    filtered_vel_y(i) = x_kalman(5);
    filtered_vel_z(i) = x_kalman(6);
%     Kg(i,:) = Kalman_gain;
    P_norm(i) = norm(cov);
end

figure;hold on;
plot(t,vel_N,'b');
plot(t,filtered_vel_x+vx_imu_NED','r')
ylabel('N velocity [m/s]')
xlabel('Time [sec]')
legend('True Trajectory','Kalman Filtered Trajectory','Location','NorthOutside')
box on; axis tight;
hold off;

figure;hold on;
plot(t,vel_E,'b');
plot(t,filtered_vel_y+vy_imu_NED','r')
ylabel('E velocity [m/s]')
xlabel('Time [sec]')
hold off;


figure;hold on;
plot(t,vel_D,'b');
plot(t,filtered_vel_z+vz_imu_NED','r')
ylabel('D velocity [m/s]')
xlabel('Time [sec]')
hold off;


figure;hold on;
plot(t,N,'b');
plot(t,filtered_x+posX_imu_NED','r')
ylabel('N position [m]')
xlabel('Time [sec]')
hold off;

figure;hold on;
plot(t,E,'b');
plot(t,filtered_y+posY_imu_NED','r')
ylabel('E position [m]')
xlabel('Time [sec]')
hold off;

figure;hold on;
plot(t,D,'b');
plot(t,filtered_z+posZ_imu_NED','r')
ylabel('D position [m]')
xlabel('Time [sec]')
hold off;

