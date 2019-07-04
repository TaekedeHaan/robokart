% Author: Taeke de Haan
% Date: 30-05-2018
clear all
close all
clc

% fix path
addpath(genpath('lib'))
addpath('symb')

%% init
% simulation
dt = 0.05;              %[s]
t_end = 1000;             %[s]
t = 0:dt:t_end;         %[s]
itterations = t_end/dt; %[-]

% plot
dim = [200, 200, 1000, 350];

% animation
fline = 20;
static = false;

par = load_param();
[La, Lb, L, m1, m2, I1, I2] = unfold_param(par);

% torque and force
force = 10;   % [N]
torque = 0;

%% init

% location backwheel
phi1 = 0; %0;
x1 = -La;%a;% 0;
y1 = 0; % 0;

% location front whee;
phi2 = 0; % pi;
x2 = x1 + Lb * cos(phi1) + La * cos(phi2); % a + b; %
y2 = y1 + Lb * sin(phi1) + La * sin(phi2); % d; % ;


q = [x1, y1, phi1, x2, y2, phi2];
qd = zeros(1, 6);

% pack
y = [q, qd];

% transform to possible space
y(1,:) = gauss_newton(y(1,:), par);

lim = [-10, 10];
f = init_animation([0,0,1000,800], lim);
set(f,'KeyPressFcn',@key_press);
l_current = [];

for i= 1:itterations
    tic
    
    % constrain
    y(i,:) = gauss_newton(y(i,:), par);
    
    % detect input
    keyPress = guidata(f);
    if ~isempty(keyPress)
        disp(keyPress);
    end
    
    
    % compute reference
    if ~isempty(keyPress)
        switch keyPress
            case 'leftarrow'
                steerRef = 15/180*pi + y(i,3); % [rad]
            case 'rightarrow'
                steerRef = -15/180*pi + y(i,3); % [rad]
            case 'uparrow'
                 velocityRef = 5; % [m/s]
                 steerRef = y(i,3);
            case 'downarrow'
                 velocityRef = 0; % [m/s]  
            otherwise
                velocityRef = 0;
                steerRef = y(i,3);
        end
    else
         steerRef = y(i,3);
        velocityRef = 0;
    end
    
    % compute control action
    torque(i) = 0.01 * (steerRef  - y(i,6));
    v = sqrt(y(i,7)^2 + y(i,8)^2); 
    force(i) = 100 * (velocityRef  - v);    
    
    % saturate if above capabilities
    torque(i) = max(min(torque(i), par.torqueMax), par.torqueMin);
    force(i) = max(min(force(i), par.forceMax), par.forceMin);
    
    [yd(i,:), lambda(i,:)] = compute_system(y(i,:), par, force(i), torque(i));
    
    % intergrate numericly
    y(i + 1,:) = int_rk4(y(i,:), dt, par, force(i), torque(i));
    
    % unpack
    q = y(i,1:4);
    
    l_current = update_animation(f, l_current, y(i,:));
    
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

i = i + 1;
y(i,:) = gauss_newton(y(i,:), par);
[yd(i,:), lambda(i,:)] = compute_system(y(i,:), par, force, torque);


%% compute enrgy
% for i = 1:(itterations + 1)
%     % determine enrgy
%     Ekin(i) = 1/2 * y(i,7:end) * M * y(i,7:end)';
%
%     diff(i) = y(i,3) - y(i,6); % phi1 - phi2
%
%     if i ~= 1
%         W(i) = torque(i) * (diff(i) - diff(i-1));
%     else
%         W(i) = 0;
%     end
% end
%
% W = cumsum(W);

%% animate
% animate_bar(A, B1, B2 , C, dt, fline, static)
% saveas(gca, 'fig/animate', 'jpg')
% saveas(gca, 'fig/animate', 'epsc')


%% Path of A and B
% figure('Position', dim)
% hold on
% plot(A(:,1), A(:,2))
% plot(C(:,1), C(:,2))
% legend('A', 'C')
% xlabel('x [m]')
% ylabel('y [m]')
% title('Position of EzyRoller Over Time')
% saveas(gca, 'fig/path', 'jpg')
% saveas(gca, 'fig/path', 'epsc')

%% anguylar velocities of body 1 and 2
figure('Position', dim)
plot(t, y(:,[9, 12]))
legend('\phi_1d', '\phi_2d')

title('Angular velocities')
xlabel('time [s]')
ylabel('Angular velocity [rad/s]')

saveas(gca, 'fig/ang_vel', 'jpg')
saveas(gca, 'fig/ang_vel', 'epsc')


%% Linear velocities of body 1 and 2
figure('Position', dim)
plot(t, y(:,[7, 8, 10, 11]))
legend('x_1d', 'y_1d', 'x_2d', 'y_2d')

title('Linear velocities')
xlabel('time [s]')
ylabel('Linear velocity [n/s]')

saveas(gca, 'fig/lin_vel', 'jpg')
saveas(gca, 'fig/lin_vel', 'epsc')
