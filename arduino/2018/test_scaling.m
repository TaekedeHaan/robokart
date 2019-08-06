close all
clc
clear all

t = 0;
goal = 1;
value = 0;
dt = 20;
scalingIncrement = 0.025;

for i = 1:100
    t(i + 1) = t(i) + dt;
    value(i+1) = value(i) + scalingIncrement * (goal - value(i));
end
t = t / 1000;
figure
plot(t, value)