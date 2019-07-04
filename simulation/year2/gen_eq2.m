%% Generate equations
% Author: Taeke de Haan
% Date: 27-03-2017

clear all; close all; clc

%% Constants (symbolic definition)
% length
syms a   real

% mass
syms m1 I1   real

% other
syms g torque force           real

%% Generalized coordinates and their derivatives
syms x1 y1 phi1 real
syms x1d y1d phi1d real
syms x1dd y1dd phi1dd real

q   = [x1 y1 phi1].';
qd  = [x1d y1d phi1d].';
qdd  = [x1dd y1dd phi1dd].';

%% Mapping
% center of mass positions as function of the generalized coordinates

% holonomic constraints
% -

% non-holonomic constraints
S1 = - x1d * sin(phi1) + y1d * cos(phi1) - a * phi1d;

% mass
M = [m1, m1, I1];
M = diag(M);

%% mapping
% T = [x2 y2 phi2 x3 y3 phi3 x4 y4 phi4 x5 y5 phi5, x6 y6 phi6].';
% dT_dq = simplify(jacobian(T,q));
% Td = dT_dq*qd; 

% Determine T2
% T2 = jacobian(Td,q)*qd;
% T2 = simplify(T2);

%% constraints
% compute derivatives
S = S1;
Sq  = simplify(jacobian(S,qd));
Sd = Sq * qd; % this is dC/dt=dC/dx*xd

% and next the convective terms d(dC/dt)/dx*xd
% S2 = jacobian(S2,q)*qd;
% S2 = simplify(S2);

S2 = simplify(jacobian(Sd,q) * qd);
% ddt_dSdqd = simplify(jacobian(dS_dqd,q))*qd;

%% EOM in matrix form
% gen mass
% M_bar = dT_dq' * M * dT_dq;
% M_bar_text = simplify(M_bar);

% gen forces
F = [0, 0, 0]';

% zero matrix
O = zeros(size(Sq,1),size([M, Sq'],2) - size(Sq,2));

% Compute answer
result = [M Sq'; Sq, O] \ [F; -S2];
qdd = simplify(result(1:3));
lambda = simplify(result(4:length(result)));

% export to latex
% disp(latex_pretify(M_bar_text));
% disp(latex_pretify(Q_bar_text));

%% Save symbolic derivation to script file.
% Use Diary function, save the symbolicly derived functions to file. 
% Q_bar

% qdd
if exist('symb\symb_qdd.m', 'file')
    ! del symb\symb_qdd.m
end
diary symb\symb_qdd.m
    disp('qdd = ['), disp(qdd), disp('];');
diary off

% C
if exist('symb\symb_C.m', 'file')
    ! del symb\symb_C.m
end
diary symb\symb_C.m
    disp('C = ['), disp(C), disp('];');
diary off

% Cq
if exist('symb\symb_Cq.m', 'file')
    ! del symb\symb_Cq.m
end
diary symb\symb_Cq.m
    disp('Cq = ['), disp(dC_dq), disp('];');
diary off

% lambda
if exist('symb\symb_lambda.m', 'file')
    ! del symb\symb_lambda.m
end
diary symb\symb_lambda.m
    disp('lambda = ['), disp(lambda), disp('];');
diary off