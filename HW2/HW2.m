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

%% 4
DCM = TaitBryan2DCM(-3.222, -9.316, 96.061)

%% 5
p1 = [5.7; 2.5];
theta = deg2rad(-30);
R_c = [cos(theta), -sin(theta); sin(theta), cos(theta)];
theta_p = -theta;
R = [cos(theta_p), -sin(theta_p); sin(theta_p), cos(theta_p)];
p2 = R * p1


x1 = [0, 10; 0, 0];
y1 = [0, 0; 0, 10];
x2 = R_c * x1;
y2 = R_c * y1;
plot(x1(1,:), x1(2,:), 'rx-')
hold on;
plot(y1(1,:), y1(2,:), 'rx-')
plot(x2(1,:), x2(2,:), 'bx:')
plot(y2(1,:), y2(2,:), 'bx:')
plot(p1(1), p1(2), 'ro');
plot(p2(1), p2(2), 'bo');
hold off;
axis equal