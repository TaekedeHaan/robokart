close all
clear all
clc

label = 'mech_heavy_low_trans';

% mechanical
r = 0.065/2; %[m] wheel redius
m = 3.0; %[kg]

% lijkt ergens tussen de 1:9 en 1:12 te liggen
n = 2 * 2.5/1; % [-] transmission
mu = 0.15; % [-] losses in total mechanical power train

save(label)