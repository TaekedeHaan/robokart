clear all
close all
clc

% ask user input
motor = input('Do you want to select [1] BLDC_sys_4000KV_60A, [2] BLDC_mot_2060KV_140A ');

% file to load
switch motor
    case 1
        label = 'BLDC_sys_4000KV_60A';
    case 2
        label = 'BLDC_mot_2060KV_140A';
    otherwise
        error('unkown motor selected');
end

%% init 
% data path
dataPath = [pwd, filesep, 'data', filesep];
load([dataPath, label]);

% constants
g = 9.81; %[m/s^2]

% car specs
m = 2.7; %[kg]
r = 0.05; %[m]
n = 1; % [-] transmission
mu = 0.10; % [-] losses in total mechanical power train

% loop
dt = 0.01;
t = 1:dt:100; %[s]

%% to SI
kv = 2 * pi * kv /60; %[rad/s / V]

%% determine maximum
% maximum velosity velocity
wMax = kv * V * n; % [rad/s]
vMax = r * wMax; % [m/s]

% maximum torque
kt = 1 / kv; %[Nm/A]
TMax = kt * I; %[Nm]

% generate vectors over an interval
wInt = linspace(0, wMax, 10000); %[rad/s]
TInt = linspace(TMax, 0, 10000); %[Nm]
PInt = wInt .* TInt; % [W]

% Losses
Ploss = R * I^2; %[W]

%% numerical intergration since im lazy
% initial conditions
s(1) = 0; %[m]
v(1) = 0; %[m/s]
i = 1; % counter

% compute position over time
for tCurr = t
    
    % determine where we are on the torque-velovity curve
    w(i) = v(i)/r;
    [wError, loc] = min(abs(wInt-w(i))); 
    
    % loss due to friction in power train
    TRres = m * g * mu * r; % [Nm]
    
    % friction cannot make us move backwards
    if TInt(loc) > TRres
        T(i) = TInt(loc) - TRres; %[Nm]
    else
        T(i) = 0; %[Nm]
    end
    
    % Determine acceleration
    F(i) = T(i) / r; %[N]
    a(i) = F(i)/m; %[m/s^2]
    
    % apply inergration
    v(i + 1) = v(i) + a(i) * dt;
    s(i + 1) = s(i) + v(i) * dt;
    
    % increment counter
    i = i + 1;
end

% show result
disp(['Zero torque velosity: ', num2str(vMax), '[m/s]']);
disp(['Maximum torque at ', num2str(I),  '[A]: ',num2str(TMax), ' [Nm]']);
disp(['Maximum acceleration ', num2str(I),  '[A]: ',num2str(a(1)), ' [m/s^2]']);
disp(['Maximum power losss due to resistance ', num2str(I),  '[A]: ',num2str(Ploss), ' [W]']);

%% plot results
figure('Position', [200, 200, 500, 400], 'Name', 'torque-velosity')

% torque
yyaxis left
plot(wInt, TInt)
ylabel('Torque [Nm]')
hold on

% poser
yyaxis right
plot(wInt, PInt)
ylabel('Power [W]')
xlabel('velocity [rad/s]')

grid minor
title('Torque Power vs Velosity')

saveas(gcf, [pwd, filesep, 'fig',filesep, char(label), '_T-w.png'])

%%
figure('Position', [200, 200, 500, 400], 'Name', 'position-time')

yyaxis left
plot(t, s(1:length(t)))
ylabel('position [m]')

yyaxis right
plot(t, v(1:length(t)))
ylabel('Velosity [m/s]')
xlabel('time[s]')

grid minor
title('Posiotion and Torque Over Time')
saveas(gcf, [pwd, filesep, 'fig',filesep, char(label), '_time.png'])