%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% apply_kalman2
%
% Applies a Kalman filter to noisy samples of aircraft vertical velocity
% (climb rate) by calling the Kalman filter function (which you will
% create).
% To use this function in the HW exercise, you will need to: 1) update the
% file paths, based on your local directory structure; and 2) replace the 
% line of code that calls the kalman filter function with a call to your 
% own Kalman filter function.
%
% C. Parrish, 11/24/2015
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;

% Editable filenames %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
velFile = strcat('E:\desktop_files\01Chris_Desktop\OSU\', ...
    'Kinematic Positioning and Navigation Class\' ,...
    'Homework Exercises\HW6\noisy_ngs_vert_vel.mat');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load(velFile);    % Contains the noisy samples of vertical velocity

tIndx = 2:2:length(vel_samples)*2;   % Sample interval is 2 sec

% Now apply the Kalman filter
for i =1:length(vel_samples)
    xSample = vel_samples(i);
    
    % Note: in the line of code below, you should replace the call to
    % "kalman_traj_hw5) with a call to the Kalman filter function you write
    % for this HW
    [x_kalman, cov, Kalman_gain] = kalman_traj_hw5(xSample);
    
    filtered_h_el(i) = x_kalman(1);
    filtered_vel(i) = x_kalman(2);
    Kg(i,:) = Kalman_gain;
    P_norm(i) = norm(cov);
end

% Create plots of the output trajectory
hold on
subplot(2,1,1)
plot(tIndx,filtered_h_el,'r','LineWidth',2.5)
title('Kalman Filter Results')
xlabel('Time (sec)');
ylabel('Altitude (m)');
legend('Kalman filtered trajectory')
subplot(2,1,2)
plot(tIndx,filtered_vel,'g','LineWidth',2.5)
xlabel('Time (sec)');
ylabel('Velocity (m/s)');
hold on
plot(tIndx,vel_samples)
legend('Kalman filtered','Noisy Velocity Samples')
