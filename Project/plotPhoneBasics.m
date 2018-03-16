% % % Ammar's Phone % % % 
stationary = readDataCSV_SP('mlt-20180315-172636-9s_stationary.csv');
stationary.plotRotation();
saveas(gcf, 'stationary.png');
stationary.plotMagnetometer();
saveas(gcf, 'stationary_magnetometer.png');

pitch = readDataCSV_SP('mlt-20180315-173225-7s_pitch.csv');
pitch.plotRotation();
saveas(gcf, 'pitch.png');

roll = readDataCSV_SP('mlt-20180315-173125-17s_roll.csv');
roll.plotRotation();
saveas(gcf, 'roll.png');

yaw = readDataCSV_SP('mlt-20180315-172656-10s_yaw.csv');
yaw.plotRotation();
saveas(gcf, 'yaw.png');
yaw.plotMagnetometer();
saveas(gcf, 'yaw_magnetometer.png');


magnet = readDataCSV_SP('mlt-20180315-180611-13s_magnet.csv');
magnet.plotMagnetometer();
saveas(gcf, 'magnet.png');
% % % % % % % % % % % % % % % % % % % %