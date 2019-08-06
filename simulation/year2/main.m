% Author: Taeke de Haan
% Date: 30-05-2018
clear all
close all
clc

% fix path
addpath(genpath('lib'))
addpath('symb')

% TODO: the input force acts on the cart center of mass, not on the cart
% wheels, this can be fixed by adding an additionall coordinate frame
% TODO: the fornt wheel of the cart is connected to the rest via a
% constrained. I think a faster and neater solution is to just use the
% (simple) kinematics to constrain the wheels
% TODO: how are we going to model slipping?

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
[La, Lb, L, ma, mb, m, Ia, Ib, I, b] = unfold_param(par);

% torque and force
force = 10;   % [N]
torque = 0;

%% init

% init
phi = 0; %
x = 0;%
y = 0; % 
alpha = 0;

q = [x; y; phi; alpha];
% y = get_y(q);

xd = 0;
yd = 0;
phid = 0;
alphad = 0;

qd = [xd; yd; phid; alphad];
% yd = get_yd(q, qd);

% pack
y = [q; qd];

% transform to possible space
% y(1,:) = gauss_newton(y(1,:), par);

lim = [-15, 15];
f = init_animation([0,0,1000,800], lim);
set(f,'KeyPressFcn',@key_press);
l_current = [];

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
    if ~isempty(keyPress)
        switch keyPress
            case 'leftarrow'
                steerRef = 15/180*pi; % [rad]
            case 'rightarrow'
                steerRef = -15/180*pi; % [rad]
            case 'uparrow'
                 velocityRef = par.vMax; % [m/s]
                 steerRef = 0;
            case 'downarrow'
                 velocityRef = 0; % [m/s]  
            otherwise
                velocityRef = 0;
                steerRef = 0;
        end
    else
         steerRef = 0;
        velocityRef = 0;
    end
    
    % compute control action
    torque(i) = 0.05 * (steerRef  - y(4,i));
    v = sqrt(y(5,i)^2 + y(6,i)^2); 
    force(i) = 100 * (velocityRef  - v);    
    
    % saturate if above capabilities
    torque(i) = max(min(torque(i), par.torqueMax), par.torqueMin);
    force(i) = max(min(force(i), par.forceMax), par.forceMin);
    
    % [yd(i,:), lambda(i,:)] = compute_system(y(i,:), par, force(i), torque(i));
    [tTemp, yTemp] = ode45(@(t,y)get_acceleration_system(t, y, force(i), torque(i)),[t(i), t(i+1)],y(:,i));
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

%% anguylar velocities of body 1 and 2
figure('Position', dim)
plot(t, y([9, 12],:))
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

