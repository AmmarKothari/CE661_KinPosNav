function [velocity,position,t] = pos_vel_from_IMU(Accel,t0,velocity0,position0,freq)

position(1) = position0;
velocity(1) = velocity0;
t(1) = t0;
dt = 1 / freq;
for i=2:size(Accel,1)
    t(i) = (i-1)/freq;
    velocity(i,1) = velocity(i-1) + Accel(i-1) * dt; % Velocity
    position(i,1) = position(i-1) + velocity(i-1,1) * dt + Accel(i-1) * dt^2/2; % Position
end


return