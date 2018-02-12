t = linspace(0,2*pi,1000);
s1 = sin(t);
s2 = sin(t+pi/2);
plot(t,s1, 'bx', t, s2, 'r+')
[acor, lag] = xcorr(s1,s2);
[~,I] = max(abs(acor));
lagDiff = lag(I) % this is the number of samples that the second signal is offset from the first

figure(2)
plot(lag,acor) % given an amount of lag, how much correlation is there, peak here tells you signal offset
a3 = gca;
a3.XTick = sort([-3000:1000:3000 lagDiff]);

figure(3)
plot(t,s1, 'bx', t(lagDiff:end), s1(lagDiff:end), 'r+')