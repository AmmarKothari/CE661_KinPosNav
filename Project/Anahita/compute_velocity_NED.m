function [vel] = compute_velocity_NED(prmt)

for i=2:length(prmt)
    vel(i,1) = prmt(i) - prmt(i-1);
end

idx = find(abs(vel) < 1e-8 & abs(vel) ~= 0);
vel(idx) = 0;

idx = find(vel ~= 0);
for i=1:length(idx)-1
    vel(idx(i)+1:idx(i+1)-1) = vel(idx(i));
end

vel(idx(end):end,1) = vel(idx(end));


return