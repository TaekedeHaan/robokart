% https://hobbyking.com/en_us/sp-036740-01-rc-car-motors-2250kv.html
clear all
close all
clc

label = 'BLDC_surpass';
link = 'https:https://hobbyking.com/en_us/sp-036740-01-rc-car-motors-2250kv.html';

% electro
I = 80; % [A] %45A controller, 60A motor
nCellMax = 4; %[2 - 3 cell Lipo]
P = nan; %[W] (unknown)

% motor specs
kv = 2250; % 4000; %[RPM/V]
R = 0.0129; %[Ohm] rough estimate

save(label)