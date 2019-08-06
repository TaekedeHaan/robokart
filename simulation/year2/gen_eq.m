%% Generate equations
% Author: Taeke de Haan
% Date: 07-07-2019
clear all
close all
clc

%% init parameters
% parameters (parametric)
% syms La Lb L   real
% syms ma mb m Ia Ib I bLin bRot fFricMax  real
% syms g
syms f1 f2 f3 f4 f5 f6 f7 f8 f9 real

% parameters (numeric) (tipically off)
par = load_param();
[La, Lb, L, ma, mb, m, Ia, Ib, I, bLin, bRot, fFricMax] = unfold_param(par);
g = 9.81;

% control input
syms torque force           real

% Generalized coordinates and their derivatives
syms xCoM yCoM phi alpha real
syms xCoMd yCoMd phid alphad real
syms xCoMdd yCoMdd phidd alphadd real

q   = [xCoM yCoM phi alpha].';
qd  = [xCoMd yCoMd phid alphad].';
qdd  = [xCoMdd yCoMdd phidd alphadd].';

%% Mapping
% center of mass positions as function of the generalized coordinates
% body 1
x1 = xCoM;
y1 = yCoM;
phi1 = phi;

% body 2
xa = x1 + La*cos(phi1);
ya = y1 + La*sin(phi1);
phia = phi1 + alpha;

% body 3
xb = x1 - Lb*cos(phi1);
yb = y1 - Lb*sin(phi1);
phib = phi1;

T = [x1, y1, phi1, xa, ya, phia, xb, yb, phib];
Tq = jacobian(T, q);
Td = jacobian(T, q)*qd;
Tdd = jacobian(T, q)*qd + jacobian(T, qd)*qdd;

% non-holonomic constraints
S1 = Td(4) * sin(phia) - Td(5) * cos(phia); % tire front
S2 = Td(7) * sin(phib) - Td(8) * cos(phib); % tire back

% mass
M = [m, m, I, ma, ma, Ia, mb, mb, Ib];
M = diag(M);
Mbar = simplify(Tq' * M * Tq);

%% Differentiate constraints
% non holonomic
S = [S1; S2];
Sq  = simplify(jacobian(S,qd));
Sd = Sq * qd; % this is dC/dt=dC/dx*xd
S2 = simplify(jacobian(Sd,q) * qd);

%%
% vaPerp = -Td(4)*sin(phia) + Td(5)*cos(phia); 
% vbPerp = -Td(7)*sin(phib) + Td(8)*cos(phib); 
% bSlip = 0;
% fax = - bSlip*vaPerp*sin(phia);
% fay = - bSlip*vaPerp*cos(phia);
% 
% fbx = - bSlip*vbPerp*sin(phib);
% fby = - bSlip*vbPerp*cos(phib);
% 
% %% EOM no slip
% % forces
% Fslip = [0, 0, 0, fax, fay, 0 , fbx, fby, 0]
FnoSlip = [0, 0, 0, 1/2*cos(phia)*force, 1/2*sin(phia)*force, torque, 1/2*cos(phib)*force, 1/2*sin(phib)*force, 0].';
Q = [-bLin*xCoMd -bLin*yCoMd, 0, -bRot*alphad]';
g = zeros(9,1);

% zero matrix
O = zeros(length(S),length(S));

LHS = [Mbar, Sq.'; Sq, O];

% Compute answer
result = LHS \ [Q + Tq'*(FnoSlip - M * g); -S2];
qdd = simplify(result(1:length(q)))';
lambda = simplify(result(length(q)+1:end));

%% EOM slip
% FSlip = FnoSlip + [0, 0, 0, -fFricMax*sin(phia), fFricMax*cos(phia), 0, -fFricMax*sin(phib), fFricMax*cos(phib), 0]';
F = [f1, f2, f3, f4, f5, f6, f7, f8, f9].';
qddSlip = simplify(Mbar\(Q + Tq'*(F - M * g)))';
lambdaSlip = [fFricMax, fFricMax];


%% Save symbolic derivation to script file.
% Use Diary function, save the symbolicly derived functions to file. 

% x
if exist('symb\get_x.m', 'file')
    ! del symb\get_x.m
end
diary symb\get_x.m
    disp('function x = get_x(q)')
    
    disp('% get parameters')
    disp('par = load_param();');
    disp('[La, Lb, L, ma, mb, m, Ia, Ib, I, bLin, bRot, fFricMax] = unfold_param(par);');
    
    disp('xCoM = q(1);')
    disp('yCoM = q(2);')
    disp('phi = q(3);')
    disp('alpha = q(4);')
    
    disp('x = ['), disp(T'), disp('];');
diary off

% xd
if exist('symb\get_xd.m', 'file')
    ! del symb\get_xd.m
end
diary symb\get_xd.m
    disp('function xd = get_xd(q, qd)')
    disp('% get parameters')
    disp('par = load_param();');
    disp('[La, Lb, L, ma, mb, m, Ia, Ib, I, bLin, bRot, fFricMax] = unfold_param(par);');
    disp('');
    disp('xCoM = q(1);')
    disp('yCoM = q(2);')
    disp('phi = q(3);')
    disp('alpha = q(4);')
    disp('');    
    disp('xCoMd = qd(1);')
    disp('yCoMd = qd(2);')
    disp('phid = qd(3);')
    disp('alphad = qd(4);')
    
    disp('xd = ['), disp(Td'), disp('];');
diary off

% qdd
if exist('symb\get_qdd_no_slip.m', 'file')
    ! del symb\get_qdd_no_slip.m
end
diary symb\get_qdd_no_slip.m
    disp('function qdd = get_qdd_no_slip(y, force, torque, par)')
    
    disp('%unpack')
    disp('[La, Lb, L, ma, mb, m, Ia, Ib, I, bLin, bRot, fFricMax] = unfold_param(par);');
    disp('[xCoM, yCoM, phi, alpha, xCoMd, yCoMd, phid, alphad] = unfold_y(y);');
    disp('')
    disp('qdd = ['), disp(qdd'), disp('];');
    
diary off

if exist('symb\get_qdd_double_slip.m', 'file')
    ! del symb\get_qdd_double_slip.m
end
diary symb\get_qdd_double_slip.m
    disp('function qdd = get_qdd_double_slip(y, F, par)')
    
    disp('%unpack')
    disp('[La, Lb, L, ma, mb, m, Ia, Ib, I, bLin, bRot, fFricMax] = unfold_param(par);');
    disp('[xCoM, yCoM, phi, alpha, xCoMd, yCoMd, phid, alphad] = unfold_y(y);');
    disp('[f1, f2, f3, f4, f5, f6, f7, f8 ,f9] = unfold_F(F);')
    disp('')  
    disp('qdd = ['), disp(qddSlip'), disp('];');
diary off


% qddSlip = Mbar\(Q + Tq'*(FSlip - M * g));
% lambdaSlip = [fFricMax, fFricMax];

% lambda
if exist('symb\get_forces_system.m', 'file')
    ! del symb\get_forces_system.m
end
diary symb\get_forces_system.m
    disp('function lambda = get_forces_system(t, y, force, torque)')
    
    disp('par = load_param();');
    disp('[La, Lb, L, ma, mb, m, Ia, Ib, I, bLin, bRot, fFricMax] = unfold_param(par);');
    
    
    disp('%unpack y')
    disp('[xCoM, yCoM, phi, alpha, xCoMd, yCoMd, phid, alphad] = unfold_y(y);');

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