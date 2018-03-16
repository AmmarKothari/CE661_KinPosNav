function [DCM_body_to_local] = body2NED_frame(roll,pitch,yaw)

tetha = pitch;
phi = roll;
psi = yaw;

DCM_body_to_local(1,1) = cos(tetha)*cos(psi);
DCM_body_to_local(1,2) = -cos(phi)*sin(psi)+sin(phi)*sin(tetha)*cos(psi);
DCM_body_to_local(1,3) = sin(phi)*sin(psi)+cos(phi)*sin(tetha)*cos(psi);
DCM_body_to_local(2,1) = cos(tetha)*sin(psi);
DCM_body_to_local(2,2) = cos(phi)*cos(psi)+sin(phi)*sin(tetha)*sin(psi);
DCM_body_to_local(2,3) = -sin(phi)*cos(psi)+cos(phi)*sin(tetha)*sin(psi);
DCM_body_to_local(3,1) = -sin(tetha);
DCM_body_to_local(3,2) = sin(phi)*cos(tetha);
DCM_body_to_local(3,3) = cos(phi)*cos(tetha);

% Rx_roll = [1 0 0;...
%     0 cos(roll) sin(roll);...
%     0 -sin(roll) cos(roll)];
% 
% Ry_pitch = [cos(pitch) 0 -sin(pitch);...
%     0 1 0;...
%     sin(pitch) 0 cos(pitch)];
% 
% Rz_yaw = [cos(yaw) sin(yaw) 0;...
%     -sin(yaw) cos(yaw) 0;...
%     0 0 1];
% 
% DCM_body_to_local = Rx_roll * Ry_pitch * Rz_yaw;


return