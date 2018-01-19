% HW1.m
%
% This script is to assist with HW assignment #1 in Kinematic Positioning 
% and Navigaiton. It reads a post-processed trajectory (blended navigation
% solution) generated from an OxTS xNAV system, processed in RT
% Post-Process, and saved .csv file format. To use this script, you will
% need to modify the input file path and file name. Additionally, if you
% have different fields (or a different order of the fields) in your csv 
% file, you will need to modify the column numbers.
% 
% C. Parrish, 10/20/2014 (Christopher.Parrish@oregonstate.edu)
% Modified: 1/9/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;

% User-entered parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pathName = strcat('./');

csvFname = '180108_202208.csv';  

rollCol = 17;          % Column # in the csv file of the roll field
pitchCol = 16;         % Column # in the csv file of the pitch field
headingCol = 15;        % Column # in the csv file of the heading field
sigmaRollCol = 26;     % Column # of sigma_roll
sigmaPitchCol = 25;    % Column # of sigma_pitch
sigmaHeadingCol = 24;  % Column # of sigma Heading
speed3DCol = 11;           % Column # of 3D speed

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

myTrajectory = csvread(strcat(pathName,csvFname),1,2);

% Assign fields to variables
roll = myTrajectory(:,rollCol);
pitch = myTrajectory(:,pitchCol);
heading = myTrajectory(:,headingCol);
sigmaRoll = myTrajectory(:,sigmaRollCol);
sigmaPitch = myTrajectory(:,sigmaPitchCol);
sigmaHeading = myTrajectory(:,sigmaHeadingCol);
speed3D = myTrajectory(:, speed3DCol);

% Compute stats
meanRoll = mean(roll);
meanPitch = mean(pitch);
meanHeading = mean(heading);
meanSpeed = mean(speed3D);
stdRoll = std(roll);
stdPitch = std(pitch);
stdHeading = std(heading);
stdSpeed = std(speed3D);

%% Create plots
figure
plot(roll)
title('roll')
export_fig  'Roll.png'
figure
plot(pitch)
title('pitch')
export_fig  'Pitch.png'
figure
plot(heading)
title('heading')
export_fig 'Heading.png'
figure
plot(sigmaRoll)
title('\sigma_{Roll}')
export_fig  'StDev_Roll.png'
figure
plot(sigmaPitch)
title('\sigma_{Pitch}')
export_fig  'StDev_Pitch.png'
figure
plot(sigmaHeading)
title('\sigma_{Heading}')
export_fig  'StDev_Heading.png'