function [par] = load_param()

%
par.r = 0.04; % [m]

% dim
par.La = 0.15;  % [m]
par.Lb = 0.15;  % [m]
par.L = par.La + par.Lb;

% mass/ inirtia
par.m = 5;     % 
par.ma = 0.2;   % [kg]
par.mb = 0.2;   % [kg]

par.I = 1/12*par.m*par.L^2;     % [kgm2]
par.Ia = 1/12*par.ma*(2 * par.r)^2;
par.Ib = 1/12*par.mb*(2 * par.r)^2;

par.torqueMax = 0.8; % [Nm]
par.torqueMin = -0.8; % [Nm]


T = 2; % [Nm]
par.forceMax = T/par.r; % [N]
par.forceMin = -T/par.r; % [N]
end