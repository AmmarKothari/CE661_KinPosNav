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
P(1) = x_0;
for i = 1:size(M,1)
    V(i+1) = V(i) + dt*M(i,1);
    P(i+1) = P(i) + dt*V(i+1) + 1/2*dt*dt*M(i,1);
end

Names = {'Position', 'Velocity', 'Acceleration'};
Data = {P, V, M(:,1)};
Units = {'m', 'm/s', 'm/s^2'};
Colors = {'r-', 'b-', 'g-'};
for i = 1:3
    figure(i)
    plot(t(1:length(Data{i})), Data{i}, Colors{i},'DisplayName',Names{i});
%     legend(gca,'show')
    title([Names{i}, ' from Accelerometer Data'])
    xlabel('Time (s)');
    ylabel(sprintf('%s (%s)', Names{i}, Units{i}));
    saveas(gcf, strcat(Names{i},'.png'));
    close(gcf);
end

%% Unbounded error
bias = 70*10^-6;
V_error = zeros(size(M,1)+1,1); % velocity
P_error = zeros(size(M,1)+1,1); % position

for i = 1:size(M,1)
    V_error(i+1) = V_error(i) + dt*bias;
    P_error(i+1) = P_error(i) + dt*V_error(i) + 1/2*dt*dt*bias;
end
Data = {P_error, V_error, M(:,1)};
for i = 1:2
    figure(i)
    plot(t(1:length(Data{i})), Data{i}, Colors{i},'DisplayName',Names{i});
    title(['Error in ', Names{i}, ' from Accelerometer Bias'])
    xlabel('Time (s)');
    ylabel(sprintf('%s Error (%s)', Names{i}, Units{i}));
    saveas(gcf, strcat(Names{i},'_error.png'));
    close(gcf);
end

%% Down Direction
down = mean(M(:,3));