function [x_kalman, cov, Kalman_gain] = kalman_filter(obs,lat,lon,h,delta_t,vel_N,vel_E,vel_D,roll,pitch,yaw,sigma_Q,sigma_R)

lat = deg2rad(lat);
lon = deg2rad(lon);
w_e = 7.27220521664304e-05;
a = 6378137.0;
R_e = 6371.0088e+3;
f = 1/298.257223563;
e = 2*f-f^2;
N = a / sqrt(1-e^2*sin(lat)^2);
M = (a*(1-e^2)) / (1-e^2*sin(lat)^2)^(3/2);
a1 = 9.7803267715; a2 = 0.0052790414; a3 = 0.0000232718;
a4 = -0.0000030876910891; a5 = 0.0000000043977311;
a6 = 0.0000000000007211;
gama = a1*(1+a2*sin(lat)^2+a3*sin(lat)^4)+(a4++a5*sin(lat)^2)*h+a6*h^2;
[DCM_body_to_local] = body2NED_frame(deg2rad(roll),deg2rad(pitch),deg2rad(yaw));
persistent P Phi H Q R x

%-->> Initial values for x and P
if isempty(Phi)
    I = eye(6,6);
    F_rr = [0 0 -vel_N/(M+h)^2;...
        vel_E*sin(lat)/((N+h)*cos(lat)^2) 0 -vel_E*sin(lat)/((N+h)^2*cos(lat));...
        0 0 0];
    F_rv = [1/(M+h) 0 0;...
        0 1/((N+h)*cos(lat)) 0;...
        0 0 -1];
    F_vr = [-2*vel_E*w_e*cos(lat)-vel_E^2/((N+h)*cos(lat)^2) 0 -vel_N*vel_D/(M+h)^2+vel_E^2*tan(lat)/(N+h)^2;...
        2*w_e*(vel_N*cos(lat)-vel_D*sin(lat))+vel_E*vel_N/((N+h)*cos(lat)^2) 0 -vel_E*vel_D/(N+h)^2-vel_N*vel_E*tan(lat)/(N+h)^2;...
        2*vel_E*w_e*sin(lat) 0 vel_E^2/(N+h)^2+vel_N^2/(M+h)^2-2*gama/(R_e+h)];
    F_vv = [vel_D/(M+h) -2*w_e*sin(lat)-2*vel_E*tan(lat)/(N+h) vel_N/(M+h);...
        2*w_e*sin(lat)+vel_E*tan(lat)/(N+h) (vel_D+vel_N*tan(lat))/(N+h) 2*w_e*cos(lat)+vel_E/(N+h);...
        -2*vel_N/(M+h) -2*w_e*cos(lat)-2*vel_E/(N+h) 0];
    F = [F_rr F_rv;F_vr F_vv];
    Phi = I+F*delta_t;
    % Model:  h(k) = h(k-1) + dt*h_dot(k-1)
    %         h_dot(k) = h_dot(k-1)
%     H = [0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];
    H = eye(6,6);
    Q = sigma_Q * eye(6,6);
    R = sigma_R * eye(6,6);
    x = [1 1 1 0.5 0.5 0.5]'; % State vector: [delta_pos delta_vel]'
    P = 13 * eye(6,6);
end
G = zeros(6,6); G(4:6,1:3) = DCM_body_to_local;
% Update covariance matrix of measurement transition error
Q_new = Phi * G * Q * G' * Phi' * delta_t;
Q = Q_new;
%-->> Prediction of state vector [x(-)]
x_pred = Phi * x;

%-->> Prediction of covariance matrix [P(-)]
P_pred = Phi * P * Phi' + Q;

%-->> Computing Kalman gain based on the predicted covariance matrix [P(-)],
%     state-to-measurement matrix [H], and measurement noise [R]
Kalman_gain = P_pred * H' * inv(H*P_pred*H'+R);

%-->> Correction of state vector [x(+)]
x_kalman = x_pred + Kalman_gain * (obs - H*x_pred);
x = x_kalman;

%-->> Correction of covariance matrix [P(+)]
cov = P_pred - Kalman_gain * H * P_pred;
P = cov;

return