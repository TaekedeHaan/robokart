clear all
close all
clc

%% init constants
m = 5; %[kg]

r = 0.05; %[m]
%w = 20000; %[RPM]
n = 1; % [-] transmission

% electro
kv = 2050; % 4000; %[RPM/V]

R = 0.0006; %[Ohm]
I = 140; %60; %[A]
V = 15; %7.2; % [V] input voltage
P = 2100; %[W]


%% to SI
%w = 2 * pi * w / 60; %[rad/S]
kv = 2 * pi * kv /60; %rad/s]

%% find velocity
w = kv * V * n; % [rad/s]
v = r * w; % [m/s]

% 
Ploss = R * I^2; %[W]
kt = 1 / kv; %[Nm/A]

T = kt * I; %[Nm]

F = T / r; %[N]
a = F/m; %[m/s^2]

% show result
disp(['Zero torque velosity: ', num2str(v), '[m/s]']);
disp(['Maximum torque at ', num2str(I),  '[A]: ',num2str(T), ' [Nm]']);
disp(['Maximum acceleration ', num2str(I),  '[A]: ',num2str(a), ' [m/s^2]']);
disp(['Maximum power losss due to resistance ', num2str(I),  '[A]: ',num2str(Ploss), ' [W]']);