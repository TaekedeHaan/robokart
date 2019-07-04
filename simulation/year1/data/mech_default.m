close all
clear all
clc

label = 'mech_default';

% mechanical
r = 0.075/2; %[m] wheel redius
m = 1.838; %[kg]

% lijkt ergens tussen de 1:9 en 1:12 te liggen
n = 50/14 * 2.5/1; % [-] transmission 
mu = 0.15; % [-] losses in total mechanical power train

save(label)