function C = TaitBryan2DCM(phi,theta,psi)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Function TaitBryan2DCM                                          
%                                                                         
%  Given Tait Bryan angles as input, computes the Direction Cosine Matrix 
%  that rotates from the NED local level frame to the IMU body frame
%  Input parameters:
%     phi = roll, in deg
%     theta = pitch, in deg
%     psi = heading (AKA, yaw), in deg
%  Output
%      C = 3x3 Direction Cosine Matrix
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

% Convert Euler angles from deg to rad
phi = phi*(pi/180);
theta = theta*(pi/180);
psi = psi*(pi/180);

Cx = [[1 0 0]
    [0 cos(phi) sin(phi)]
    [0 -1*sin(phi) cos(phi)]];

Cy = [[cos(theta) 0 -1*sin(theta)]
    [0 1 0]
    [sin(theta) 0 cos(theta)]];

Cz = [[cos(psi) sin(psi) 0]
    [-1*sin(psi) cos(psi) 0]
    [0 0 1]];

C = Cx*Cy*Cz;