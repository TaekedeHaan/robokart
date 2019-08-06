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
t_end = 1000;           %[s]
t = 0:dt:t_end;         %[s]
itterations = t_end/dt; %[-]

% plot
dim = [200, 200, 1000, 350];

% refresh rate animation
fline = 20;

par = load_param();
[La, Lb, L, ma, mb, m, Ia, Ib, I, b] = unfold_param(par);

% torque and force
force = 10;   % [N]
torque = 0;

%% initialize coordinates
phi = 0;    % [rad]
x = 0;      % [m]
y = 0;      % [m]
alpha = 0;  % [rad]

xd = 0;     % [m/s]
yd = 0;     % [m/s]
phid = 0;   % [rad/s]
alphad = 0; % [rad/s]

q = [x; y; phi; alpha]; 
qd = [xd; yd; phid; alphad];

% pack
y = [q; qd];

%% initialize animation
lim = [-15, 15]; % [m] position limits used
f = init_animation([0,0,1000,800], lim);

% call back function for control
set(f,'KeyPressFcn',@key_press);

% line to be plotted
l_current = [];

%% init user input
steerRef = 0;
forceRef = 0;

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
    
    % get reference
    [steerRef, forceRef] = get_reference(keyPress, steerRef,  forceRef,  par);
    
    % get control action
    [torque(i), force(i)] = get_control_action(steerRef, forceRef, y(:, i), par);
    
    [tTemp, yTemp] = ode45(@(t,y)get_acceleration_system(t, y, force(i), torque(i), par),[t(i), t(i+1)],y(:,i));
    y(:,i + 1) = yTemp(end,:)';
    
    lambda(:,i) = get_forces_system(t, y(:,i + 1), force(i), torque(i));
    
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

% i = i + 1;
% y(i,:) = gauss_newton(y(i,:), par);
% [yd(i,:), lambda(i,:)] = compute_system(y(i,:), par, force, torque);


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

% %% anguylar velocities of body 1 and 2
% figure('Position', dim)
% plot(t, y([9, 12],:))
% legend('\phi_1d', '\phi_2d')
% 
% title('Angular velocities')
% xlabel('time [s]')
% ylabel('Angular velocity [rad/s]')
% 
% saveas(gca, 'fig/ang_vel', 'jpg')
% saveas(gca, 'fig/ang_vel', 'epsc')
% 
% %% Linear velocities of body 1 and 2
% figure('Position', dim)
% plot(t, y(:,[7, 8, 10, 11]))
% legend('x_1d', 'y_1d', 'x_2d', 'y_2d')
% 
% title('Linear velocities')
% xlabel('time [s]')
% ylabel('Linear velocity [n/s]')
% 
% saveas(gca, 'fig/lin_vel', 'jpg')
% saveas(gca, 'fig/lin_vel', 'epsc')
% 
