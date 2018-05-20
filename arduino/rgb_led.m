close all
clear all
clc

outMax = 256;
outMin = 0;
w = 1000:1:2000;
slope = 256 / 500;
intensity = 0.95;

AVG = 1500;
MAX = 2000;
MIN = 1000;

% color function
r = outMax + slope * (w - MAX).^intensity;
g = outMax - slope * abs((-w + AVG)).^intensity;
b = outMax + slope * (-w + MIN).^intensity;

% saturate
r = min(outMax, max(outMin, r));
g = min(outMax, max(outMin, g));
b = min(outMax, max(outMin, b));

figure
hold on
axis([MIN, MAX, 0, outMax])
plot(w,b, 'b')
plot(w,g, 'g')
plot(w,r, 'r')
