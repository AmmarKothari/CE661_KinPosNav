fname = 'xnav_inertial_meas.csv';

M = csvread(fname, 1,0);
%%
sampling_rate = 100; %hz
t_0 = 0;
x_0 = 0;
v_0 = 0;
dt = 1/sampling_rate;

t = 0:dt:dt*size(M,1); % time

V = zeros(size(M,1)+1,1); % velocity
P = zeros(size(M,1)+1,1); % position

V(1) = v_0;
for i = 1:size(M,1)
    V(i+1) = V(i) + dt*M(i,1);
    P(i+1) = P(i) + 1/2*dt*dt*M(i,1);
end

plot(t, V, 'r-','DisplayName','Velocity');
hold on;
plot(t, P, 'b-','DisplayName','Position');
legend(gca,'show')
hold off
title('Position and Velocity from Accelerometer Data')

%% Unbounded error
bias = 70*10^-6;
V_error = zeros(size(M,1)+1,1); % velocity
P_error = zeros(size(M,1)+1,1); % position

for i = 1:size(M,1)
    V_error(i+1) = V_error(i) + dt*bias;
    P_error(i+1) = P_error(i) + 1/2*dt*dt*bias;
end

plot(t, V, 'r-','DisplayName','Velocity Error');
hold on;
plot(t, P, 'b-','DisplayName','Position Error');
legend(gca,'show')
hold off
title('Error in Position and Velocity from Accelerometer Bias');

%% Down Direction
down = mean(M(:,3));