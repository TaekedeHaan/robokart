function [par] = load_param()

%
par.r = 0.04; % [m]

% dim
par.La = 0.15;  % [m]
par.Lb = 0.15;  % [m]
par.L = par.La + par.Lb;

% mass/ inirtia
par.m = 5;     % 
par.ma = 0.4;   % [kg]
par.mb = 0.4;   % [kg]

par.I = 1/12*par.m*par.L^2;     % [kgm2]
par.Ia = 1/12*par.ma*(2 * par.r)^2;
par.Ib = 1/12*par.mb*(2 * par.r)^2;

par.torqueMax = 0.8; % [Nm]
par.torqueMin = -0.8; % [Nm]
velocityMax = 10*pi; % [rad/s]

T = 2; % [Nm]
par.forceMax = T/par.r; % [N]
par.forceMin = -T/par.r; % [N]

par.vMax = 40/3.6; % [m/s]
par.bLin = par.forceMax/par.vMax;
par.bRot = par.torqueMax/velocityMax;

%% reference
par.steerRefMax = 25/180*pi;
par.steerRefMin = -25/180*pi;

%% control
par.steerKp = 0.5;
par.steerKd = 0.0002;
end