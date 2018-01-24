%% 1
theta = pi;
C_NWU_NED = [1,0,0;0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)]
det_C = det(C_NWU_NED)
p_NED = [11.521; -215.633; 108.617];
p_NWU = C_NEW_NED * p_NED

%% 3
phi = 5.912;
theta = -1.013;
psi = 90.084;

[q1, q2, q3, q4] = TaitByran2quaternion(phi, theta, psi);
quat = [q1, q2, q3, q4]
C_NED_IMU = TaitBryan2DCM(phi, theta, psi)