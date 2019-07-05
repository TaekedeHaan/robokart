function [par] = load_param()

% dim
par.La = 0.15;  % [m]
par.Lb = 0.15;  % [m]
par.L = par.La + par.Lb;

% mass/ inirtia
par.m1 = 5;     % not specified
par.m2 = 0.2;   % [kg]
par.I1 = 1/12*par.m1*par.L^2;     % [kgm2]
par.I2 = 1/12*par.m2*0.1^2;

par.torqueMax = 0.8; % [Nm]
par.torqueMin = -0.8; % [Nm]

par.r = 0.04; % [m]
T = 2; % [Nm]
par.forceMax = T/par.r; % [N]
par.forceMin = -T/par.r; % [N]
end