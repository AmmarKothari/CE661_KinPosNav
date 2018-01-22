function [q1,q2,q3,q4] = TaitByran2quaternion(phi,theta,psi)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Function TaitBryan2quaternion                                          
%                                                                         
%  Given Euler angles as input, computes the quaternions: q1 through q4
%  Input parameters:
%     phi = roll, in deg
%     theta = pitch, in deg
%     psi = heading (AKA, yaw), in deg
%  Output
%      q1 through q4
%
%  Important note: this program was developed solely for the purpose of
%  enabling students in Kinematic Positioning and Navigation to complete HW
%  exercises and has not been rigorously tested. For any real-world 
%  applications, users are advised to investigate code on MATLAB Central
%  (or elesewhere) for converting between Euler angles, DCMs and
%  quaternions.
%
% C. Parrish  Christopher.Parrish@oregonstate.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Convert Tait Bryan angles from deg to rad
phi = phi*(pi/180);
theta = theta*(pi/180);
psi = psi*(pi/180);

% Convert Euler angle convention
Euler = [phi theta psi]';
C = [[0 0 1]
    [0 -1 0]
    [-1 0 0]];
R = C*Euler;

e1 = R(1);
e2 = R(2);
e3 = R(3)+pi;

q1 = cos(e1/2)*cos(e2/2)*cos(e3/2) + sin(e1/2)*sin(e2/2)*sin(e3/2);
q2 = sin(e1/2)*cos(e2/2)*cos(e3/2) - cos(e1/2)*sin(e2/2)*sin(e3/2);
q3 = cos(e1/2)*sin(e2/2)*cos(e3/2) + sin(e1/2)*cos(e2/2)*sin(e3/2);
q4 = cos(e1/2)*cos(e2/2)*sin(e3/2) - sin(e1/2)*sin(e2/2)*cos(e3/2);




