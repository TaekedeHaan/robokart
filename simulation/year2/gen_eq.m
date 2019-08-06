%% Generate equations
% Author: Taeke de Haan
% Date: 07-07-2019
clear all
close all
clc

%% Constants (symbolic definition)

% parametric
syms L La Lb   real
syms m I ma mb Ia Ib b   real

% numeric
par = load_param();
% [La, Lb, L, ma, mb, m, Ia, Ib, I, b] = unfold_param(par);

% other
syms g torque force           real

%% Generalized coordinates and their derivatives
syms x y phi alpha real
syms xd yd phid alphad real
syms xdd ydd phidd alphadd real

q   = [x y phi alpha].';
qd  = [xd yd phid alphad].';
qdd  = [xdd ydd phidd alphadd].';

%% Mapping
% center of mass positions as function of the generalized coordinates

% body 1
x1 = x;
y1 = y;
phi1 = phi;

% body 2
xa = x + La*cos(phi1);
ya = y + La*sin(phi1);
phia = phi1 + alpha;

% body 3
xb = x - Lb*cos(phi1);
yb = y - Lb*sin(phi1);
phib = phi1;

T = [x1, y1, phi1, xa, ya, phia, xb, yb, phib];
Tq = jacobian(T, q);
Td = jacobian(T, q)*qd;
Tdd = jacobian(T, q)*qd + jacobian(T, qd)*qdd;

% non-holonomic constraints
S1 = - Td(4) * sin(phia) + Td(5) * cos(phia); % tire front
S2 = - Td(7) * sin(phib) + Td(8) * cos(phib); % tire back

% mass
M = [m, m, I, ma, ma, Ia, mb, mb, Ib];
M = diag(M);

%% Differentiate constraints
% non holonomic
S = [S1; S2];
Sq  = simplify(jacobian(S,qd));
Sd = Sq * qd; % this is dC/dt=dC/dx*xd
S2 = simplify(jacobian(Sd,q) * qd);

%% EOM in matrix form
% forces
Mbar = simplify(Tq' * M * Tq);
F = [0, 0, -torque, 1/2*cos(phi1)*force, 1/2*sin(phi1)*force, torque, 1/2*cos(phi1)*force, 1/2*sin(phi1)*force, 0].';
Q = [-b*Td(1), -b*Td(2), 0, 0]';
g = zeros(9,1);

% zero matrix
O = zeros(length(S),length(S));

LHS = [Mbar, Sq.'; Sq, O];

% Compute answer
result = LHS \ [Q + Tq'*(F - M * g); -S2];
qdd = simplify(result(1:length(q)))';
lambda = simplify(result(length(q)+1:end));

%% Save symbolic derivation to script file.
% Use Diary function, save the symbolicly derived functions to file. 

% xd
if exist('symb\get_xd.m', 'file')
    ! del symb\get_xd.m
end
diary symb\get_xd.m
    disp('function xd = get_xd(q)')
    disp('% get parameters')
    disp('par = load_param();');
    disp('[La, Lb, L, ma, mb, m, Ia, Ib, I, b] = unfold_param(par);');
    
    disp('xd = qd(1);')
    disp('yd = qd(2);')
    disp('phid = qd(3);')
    disp('alphad = qd(4);')
    
    disp('xd = ['), disp(Td'), disp('];');
diary off

% x
if exist('symb\get_x.m', 'file')
    ! del symb\get_x.m
end
diary symb\get_x.m
    disp('function x = get_x(q)')
    
    disp('% get parameters')
    disp('par = load_param();');
    disp('[La, Lb, L, ma, mb, m, Ia, Ib, I, b] = unfold_param(par);');
    
    disp('x = q(1);')
    disp('y = q(2);')
    disp('phi = q(3);')
    disp('alpha = q(4);')
    
    disp('x = ['), disp(T'), disp('];');
diary off

% qdd
if exist('symb\eom.m', 'file')
    ! del symb\eom.m
end
diary symb\eom.m
    disp('function yd = eom(t, y, force, torque)')
    disp('qd = y(5:end);')

    disp('% get parameters')
    disp('par = load_param();');
    disp('[La, Lb, L, ma, mb, m, Ia, Ib, I, b] = unfold_param(par);');

    disp('%unpack')
    disp('[x, y, phi, alpha, xd, yd, phid, alphad] = unfold_y(y);');

    disp('qdd = ['), disp(qdd'), disp('];');
    disp('yd = [qd; qdd];');
diary off

% lambda
if exist('symb\get_forces_system.m', 'file')
    ! del symb\get_forces_system.m
end
diary symb\get_forces_system.m
    disp('function lambda = get_forces_system(t, y, force, torque)')
    
    disp('par = load_param();');
    disp('[La, Lb, L, ma, mb, m, Ia, Ib, I, b] = unfold_param(par);');
    
    
    disp('%unpack y')
    disp('[x, y, phi, alpha, xd, yd, phid, alphad] = unfold_y(y);');

    disp('lambda = ['), disp(lambda), disp('];');
diary off

% S
if exist('symb\symb_S.m', 'file')
    ! del symb\symb_S.m
end
diary symb\symb_S.m
    disp('S = ['), disp(S), disp('];');
diary off

% Sq
if exist('symb\symb_Sq.m', 'file')
    ! del symb\symb_Sq.m
end
diary symb\symb_Sq.m
    disp('Sq = ['), disp(Sq), disp('];');
diary off