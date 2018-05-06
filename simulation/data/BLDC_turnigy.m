% https://hobbyking.com/en_us/turnigy-xk3674-1900kv-brushless-inrunner.html
clear all
close all
clc

label = 'BLDC_turnigy';
link = 'https://hobbyking.com/en_us/turnigy-xk3674-1900kv-brushless-inrunner.html';

% electro
I = 61; % [A] %45A controller, 60A motor
nCellMax = 7; %[2 - 3 cell Lipo]
P = nan; %[W] (unknown)

% motor specs
kv = 	1900; % 4000; %[RPM/V]
R = 0.0129; %[Ohm] rough estimate

save(label)