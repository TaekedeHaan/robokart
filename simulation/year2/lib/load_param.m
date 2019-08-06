function [par] = load_param()
%% general
par.g = 9.81; % [m/s^2]

%% Wheels
% wheel radius
par.r = 0.04; % [m]

% dim
par.La = 0.15;  % [m]
par.Lb = 0.15;  % [m]
par.L = par.La + par.Lb;

% mass/ inertia
par.m = 5;     % 
par.ma = 0.1;   % [kg]
par.mb = 0.1;   % [kg]

par.I = 1/12*par.m*par.L^2;     % [kgm2]
par.Ia = 1/12*par.ma*(2 * par.r)^2;
par.Ib = 1/12*par.mb*(2 * par.r)^2;

%% steer servo
par.torqueMax = 0.2; % [Nm]
par.torqueMin = -0.2; % [Nm]

% damping (based on maximum velocity)
par.omegaMax = pi; % [rad/s]
par.bRot =  par.torqueMax/par.omegaMax* 10;

%% powertrain
T = 1.5; % [Nm]
par.forceMax = T/par.r; % [N]
par.forceMin = -T/par.r; % [N]

% damping (based on maximum velocity)
par.vMax = 30/3.6; % [m/s]
par.bLin = par.forceMax/par.vMax;

%%
par.steerRefMax = 15/180*pi;
par.steerRefMin = -15/180*pi;
par.steerRefDef = 0;

%% Wheels
% friction coeficient
mu = 1;
par.fFricMax = mu*par.m*par.g;

%% control
par.kpSteer = 5;
par.kdSteer = 0.0001;
end