close all
clear all
clc

label = 'mech_heavy';

% mechanical
r = 0.080/2; %[m] wheel redius
m = 5.0; %[kg]

% lijkt ergens tussen de 1:9 en 1:12 te liggen
n = 50/18 * 2.5/1; % [-] transmission
mu = 0.15; % [-] losses in total mechanical power train

save(label)