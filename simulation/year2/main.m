% Author: Taeke de Haan
% Date: 30-05-2018
clear all
close all
clc

% fix path
addpath(genpath('lib'))
addpath('symb')

% TODO: how are we going to model slipping?

%% init
% simulation
dt = 0.05;              %[s]
t_end = 1000;           %[s]
t = 0:dt:t_end;         %[s]
itterations = t_end/dt; %[-]

% plot
dim = [200, 200, 1000, 350];

% animation
fline = 20;
static = false;

par = load_param();
[La, Lb, L, ma, mb, m, Ia, Ib, I, b] = unfold_param(par);

% torque and force
force = 10;   % [N]
torque = 0;

%% initialize position
phi = 0; %
x = 0;%
y = 0; % 
alpha = 0;

xd = 0;
yd = 0;
phid = 0;
alphad = 0;

q = [x; y; phi; alpha];
qd = [xd; yd; phid; alphad];
y = [q; qd];

%% initialize animation
lim = [-15, 15];
f = init_animation([0,0,1000,800], lim);
set(f,'KeyPressFcn',@key_press);
l_current = [];

%% initialize reference
steerRef = 0;
velocityRef = 0;

%% main loop
for i= 1:itterations
    tic
    if ~ishandle(f)
        break
    end
    
    % constrain
    % y(i,:) = gauss_newton(y(i,:), par);
    
    % detect input
    keyPress = guidata(f);
    if ~isempty(keyPress)
        disp(keyPress);
    end
    
    % compute reference
    [steerRef, velocityRef] = compute_reference(keyPress, steerRef, velocityRef, par);
    
    % control action
    [force(i), torque(i)] = compute_control_action(steerRef, velocityRef, y(:,i), par);
    
    [tTemp, yTemp] = ode45(@(t,y)compute_acceleration_system(t, y, force(i), torque(i), par),[t(i), t(i+1)],y(:,i));
    y(:,i + 1) = yTemp(end,:)';
    
    lambda(:,i) = get_forces_system(t, y(:,i + 1), force(i), torque(i));
    
    % intergrate numericly
    % y(i + 1,:) = int_rk4(y(i,:), dt, par, force(i), torque(i));
    
    l_current = update_animation(f, l_current, y(:,i), lambda(:,i), force(i));
    
    % fix timing
    if ~(toc < dt) % if this is not true we do not meet required loop time
        warning('We do not meet sampling time');
    else % else wait
        while true
            if ~(toc < dt)
                break
            end
        end
    end
    
end