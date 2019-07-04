close all
clear all
clc

label = 'mech_heavy_high_trans';

% mechanical
r = 0.075/2; %[m] wheel redius
m = 3.0; %[kg]

% lijkt ergens tussen de 1:9 en 1:12 te liggen
n = 5 * 2.5/1; % [-] transmission %50/14
mu = 0.15; % [-] losses in total mechanical power train

save(label)