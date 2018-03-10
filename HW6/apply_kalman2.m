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
clear all; clf(gcf()); clc;

% Editable filenames %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
velFile = strcat('noisy_ngs_vert_vel.mat');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load(velFile);    % Contains the noisy samples of vertical velocity


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
N = 8;
fig_fn = cell(N,1);
Q = cell(N,1);
R = cell(N,1);
height_data = cell(N,1);
% % % 
fig_fn{1} = '1a_1.png';
Q{1} = [0.5, 0; 0, 0.0];
R{1} = 01;
% % %
fig_fn{2} = '1a_2.png';
Q{2} = [0.5, 0; 0, 0.5];
R{2} = 01;
% % %
fig_fn{3} = '1a_3.png';
Q{3} = [0.5, 0; 0, 0.9];
R{3} = 01;
% % %
fig_fn{4} = '1a_4.png';
Q{4} = [10, 0; 0, 0.5];
R{4} = 01;
% % %
fig_fn{5} = '1a_5.png';
Q{5} = [0.5, 0; 0, 0.5];
R{5} = 10;
% % %
fig_fn{6} = '1a_6.png';
Q{6} = [0.5, 0; 0, 0.5];
R{6} = 3;
% % %
fig_fn{7} = '1a_7.png';
Q{7} = [0.5, 0; 0, 0.5];
R{7} = 1e-3;
% % %
fig_fn{8} = '1a_8.png';
Q{8} = [0.5, 10; 10, 0.5];
R{8} = 1;


for i = 1:N
   height_data{i} = run_kalman(vel_samples, Q{i}, R{i}, fig_fn{i});  
   
end

figure(2);
for i = [1,2,3,4,6,7,8]
    plot(height_data{i});
     hold on;
     pause(1)
end
hold off;
legend(string([1,2,3,4,6,7,8]))