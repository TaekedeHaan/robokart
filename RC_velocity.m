clear all
close all
clc

% file to load
label = 'BLDC_sys_4000KV_60A';
% label = 'BLDC_mot_2060KV_140A';

%% init 
% data path
dataPath = [pwd, filesep, 'data', filesep];
load([dataPath, label]);

% car specs
m = 5; %[kg]
r = 0.05; %[m]
n = 1; % [-] transmission

%% to SI
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