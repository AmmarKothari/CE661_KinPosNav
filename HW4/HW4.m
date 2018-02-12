%% Question 3
c = 299792.458;
GPS_d = 20180;
Galileo_d = 23222;
GLONASS_d = 19130;

GPS_t = GPS_d / c
Galileo_t = Galileo_d / c
GLONASS_t = GLONASS_d / c

%% Question 4
A = [2.2426 -0.5117 0.6548 0.5450;
    -0.5117 0.69027 0.1870 0.0173;
     0.6548 0.5450 0.1870 0.0173;
     5.1334 3.1770 3.1770 4.1327;];
 A_diag = diag(A);
 GDOP = sqrt(sum(A_diag))
 PDOP = sqrt(sum(A_diag(1:3)))
 HDOP = sqrt(sum(A_diag(1:2)))
 VDOP = sqrt(sum(A_diag(3)))
 TDOP = sqrt(sum(A_diag(4)))
 
 %% Question 5
 M = csvread('ca_prn_code.txt');
 gs = cacode(1:37); % generates signal for all vehicles
 acor_all = size(gs,1);
 for i = 1:size(gs,1)
     [acor, lag] = xcorr(M, gs(i,:));
     acor_all(i) = max(acor);
 end
 % figure out the closest correlation
[~, sv] = max(acor_all);
[acor, lag] = xcorr(M, gs(sv,:));
[~,I] = max(abs(acor));
lagDiff = lag(I) % this is the number of samples that the second signal is offset from the first
figure(1);
plot(lag, acor); title('Cross Correlation')
a3 = gca;
a3.XTick = sort([-3000:1000:3000 lagDiff]);
figure(2);
plot(1:length(M), M, 'b', 1:length(M), gs(sv,:), 'r');
title('Original');
if lagDiff >= 0
    figure(3); plot(1:length(M), M, 'b', lagDiff:length(M), gs(sv,lagDiff:end), 'r'); title('Adjusted')
else
    figure(3); plot(1:length(M), M, 'b', 1:(length(M)+lagDiff), gs(sv,1:(end+lagDiff)), 'r'); title('Adjusted')
end

f_signal = 1.023 * 1000 * 1000; %1.023 Mhz frequency
