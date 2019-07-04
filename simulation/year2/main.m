% Author: Taeke de Haan
% Date: 30-05-2018
clear all
close all
clc

% fix path
addpath('lib')
addpath('symb')

%% init
% simulation
dt = 0.05;              %[s]
t_end = 5;             %[s]
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
x1 = -La;%a;% 0;
y1 = 0; % 0;
phi1 = 0; %0;

% location front whee;
x2 = x1 + Lb * cos(phi1) + La * cos(phi2); % a + b; %
y2 = y1 + Lb * sin(phi1) + La * sin(phi2); % d; % ;
phi2 = 0; % pi;

q = [x1, y1, phi1, x2, y2, phi2];
qd = zeros(1, 6);

% qd
% V = null([Cq;Sq]);
% AXd0 = V([1, 3], :);
% bXd0 = [x1d; phi1d];
% cXd0 = AXd0\bXd0;
% qd = (V * cXd0)';
% y1d = 0; % 5   % should be zero since angle is zero
% phi2d = 0;
% x2d = 0; %xid + 
% y2d = 0; % 0.5
% qd = [x1d y1d phi1d, x2d, y2d, phi2d];

% pack
y = [q, qd];

% transform to possible space
y(1,:) = gauss_newton(y(1,:), par);

for i= 1:itterations
    
    torque = 0.001*(rand - 0.5)*2;
    
    % constrain
    y(i,:) = gauss_newton(y(i,:), par);
    [yd(i,:), lambda(i,:)] = compute_system(y(i,:), par, force, torque);
    
    % intergrate numericly
    y(i + 1,:) = int_rk4(y(i,:), dt, par, force, torque);
    
    % unpack
    q = y(i,1:4);
    
    % store loactions
    A(i,:) = [y(i,1) - Lb * cos(y(i,3)), y(i,2) - Lb * sin(y(i,3))];
    B1(i,:) = [y(i,1) + La * cos(y(i,3)), y(i,2) + La * sin(y(i,3))];
    B2(i,:) = [y(i,4), y(i,5)];
    C(i,:) = [y(i,4), y(i,5)];
    
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
animate_bar(A, B1, B2 , C, dt, fline, static)
saveas(gca, 'fig/animate', 'jpg')
saveas(gca, 'fig/animate', 'epsc')


%% Path of A and B 
figure('Position', dim)
hold on
plot(A(:,1), A(:,2))
plot(C(:,1), C(:,2))
legend('A', 'C')
xlabel('x [m]')
ylabel('y [m]')
title('Position of EzyRoller Over Time')
saveas(gca, 'fig/path', 'jpg')
saveas(gca, 'fig/path', 'epsc')

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

