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
 gs = cacode(1:37);
 acor_all = size(gs,1);
 for i = 1:size(gs,1)
     [acor, lag] = xcorr(M, gs(i,:));
     plot(lag, acor)
     hold on;
     acor_all(i) = max(acor);
 end
 hold off
%  [~, sv] = max(acor_all);
%  [acor, lag] = xcorr(M, gs(sv,:));
%  plot(lag, acor)