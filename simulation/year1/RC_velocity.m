clear all
close all
clc

% run all data scripts
run(['data', filesep, 'BLDC_Default.m'])
run(['data', filesep, 'BLDC_sys_1900KV_120A.m'])
run(['data', filesep, 'mech_default.m'])
run(['data', filesep, 'mech_heavy.m'])
run(['data', filesep, 'mech_heavy_high_trans.m'])

% ask user input
motor = input('Do you want to select [0] BLDXDefault, [1] BLDC_sys_1900KV_120A, [2] turnigy [3] surpass, [4] BLDC_sys_4000KV_60A');
mech = input('Do you want to select [0] mech default, [1] mech heavy, [2] mech_heavy_high_trans, [3] mech_heavy_low_trans');
nCell = input('Do you want to select [1] 1 cell batery, [2] 2 cell batery, [3], 3 cell Battery, [4] 4 cell battery');

% file to load
switch motor
    case 0
        labelBLDC = 'BLDC_Default';
    case 1
        labelBLDC = 'BLDC_sys_1900KV_120A';
    case 2
        labelBLDC = 'BLDC_turnigy';
    case 3
        labelBLDC = 'BLDC_surpass';
    case 4
        labelBLDC = 'BLDC_sys_4000KV_60A';
    otherwise
        error('unkown motor selected');
end

switch mech
    case 0
        labelMech = 'mech_default';
    case 1
        labelMech = 'mech_heavy';
    case 2
        labelMech = 'mech_heavy_high_trans';
    case 3
        labelMech = 'mech_heavy_low_trans';
    otherwise
        error('unkown motor selected');
end
%% init 
% data path
dataPath = [pwd, filesep, 'data', filesep];
load([dataPath, labelBLDC]);
load([dataPath, labelMech]);
disp(I)
% constants
g = 9.81; %[m/s^2]

% car specs
VCell = 3.6; %[V/cell]

% loop
dt = 0.01;
tEnd = 5;
t = 0:dt:tEnd; %[s]

%% to SI
V = nCell * VCell; %[V]
kv = 2 * pi * kv /60; %[rad/s / V]

%% Sanity checks
if nCell > nCellMax
    warning('Over voltage!')
end

%% determine maximum
% maximum velosity velocity
wMax = kv * V / n; % [rad/s]
vMax = r * wMax; % [m/s]

% maximum torque
kt = 1 / kv; %[Nm/A]
TMax = kt * I * n; %[Nm]

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
    TRres = mu * TInt(i); % [Nm]
    
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
sSprint = 10;
[~, i] = min(abs(s - sSprint));
if i > length(t)
    tSprint = nan;
    warning('20m not reached');
else
    tSprint = t(i);
end


% show result
disp([num2str(sSprint), ' [m], in ' num2str(tSprint), '[s]']);
disp(['Zero torque velosity: ', num2str(3.6 * vMax), '[km/h]']);
disp(['Maximum reachable velosity: ', num2str(3.6 * max(v)), '[km/h]']);
disp(['Maximum torque at ', num2str(I),  '[A]: ',num2str(TMax), ' [Nm]']);
disp(['Maximum acceleration ', num2str(I),  '[A]: ',num2str(a(1)), ' [m/s^2]']);
disp(['Maximum power losss due to resistance ', num2str(I),  '[A]: ',num2str(Ploss), ' [W]']);

saveString = [num2str(motor), '_', char(labelBLDC), '_',num2str(nCell), '_',num2str(mech), '_',char(labelMech)];

%% plot results
figure('Position', [200, 200, 500, 400], 'Name', 'torque-velosity')

% torque
yyaxis left
plot(wInt, TInt, 'linewidth', 2)
ylabel('Torque [Nm]')
hold on

% poser
yyaxis right
plot(wInt, PInt, 'linewidth', 2)
ylabel('Power [W]')
xlabel('velocity [rad/s]')

grid minor
title('Torque Power vs Velosity')

saveas(gcf, [pwd, filesep, 'fig',filesep, saveString, '_T-w.png'])

%% vlocity time
figure('Position', [200, 200, 500, 400], 'Name', 'velosity-time')

yyaxis right
plot(t, a(1:length(t)), 'linewidth', 2)
ylabel('Acceleration [m/s^2]')
axis([0, max(t), 0, 40])

yyaxis left
plot(t, 3.6 * v(1:length(t)), 'linewidth', 2)
ylabel('Velosity [km/h]')

axis([0, max(t), 0, 60])

xlabel('time[s]')
grid minor
title('Posiotion and Torque Over Time')
saveas(gcf, [pwd, filesep, 'fig',filesep,saveString,  '_velosity.png'])

%% position - time plot
figure('Position', [200, 200, 500, 400], 'Name', 'Position-time')

plot(t, s(1:length(t)), 'linewidth', 2)
axis([0, max(t), 0, 40])

ylabel('Position [m]')
xlabel('time[s]')
grid minor

saveas(gcf, [pwd, filesep, 'fig',filesep, saveString, '_position.png'])