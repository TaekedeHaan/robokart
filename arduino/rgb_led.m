close all
clear all
clc

OUTMAX = 256;
OUTMIN = 0;
velWidth = 1000:1:2000;
SLOPE = 256 / 250;
intensity = 0.95;

AVG = 1500;
MAX = 2000;
MIN = 1000;

% color function
r1 = OUTMAX + SLOPE * (velWidth - MAX);
g1 = OUTMAX - SLOPE * abs(-velWidth + (AVG + MAX)/2);
b1 = OUTMAX + SLOPE * (-velWidth + AVG);

r2 = OUTMAX - SLOPE * (velWidth - MIN);
g2 = OUTMAX - SLOPE * abs(-velWidth + (AVG + MIN)/2);
b2 = OUTMAX - SLOPE * (-velWidth + AVG);


% saturate
r1 = min(OUTMAX, max(OUTMIN, r1));
g1 = min(OUTMAX, max(OUTMIN, g1));
b1 = min(OUTMAX, max(OUTMIN, b1));

r2 = min(OUTMAX, max(OUTMIN, r2));
g2 = min(OUTMAX, max(OUTMIN, g2));
b2 = min(OUTMAX, max(OUTMIN, b2));

figure
hold on
axis([MIN, MAX, 0, OUTMAX])
plot(velWidth,b1, 'b')
plot(velWidth,g1, 'g')
plot(velWidth,r1, 'r')

plot(velWidth,b2, 'b')
plot(velWidth,g2, 'g')
plot(velWidth,r2, 'r')
