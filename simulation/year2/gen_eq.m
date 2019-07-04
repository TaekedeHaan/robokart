%% Generate equations
% Author: Taeke de Haan
% Date: 30-05-2018

clear all; close all; clc

%% Pendulum Constants (symbolic definition)
% length
syms L La Lb   real

% mass
syms m1 m2 I1 I2   real

% load param
% [paramDim, paramMass] = load_param();
% [a, b, c, d] = unfold_param_dim(paramDim);
% [m1, m2, I1, I2] = unfold_param_mass(paramMass);

% other
syms g torque force           real

%% Generalized coordinates and their derivatives
syms x1 y1 phi1 x2 y2 phi2 real
syms x1d y1d phi1d x2d y2d phi2d real
syms x1dd y1dd phi1dd x2dd y2dd phi2dd real

q   = [x1 y1 phi1, x2, y2, phi2].';
qd  = [x1d y1d phi1d, x2d, y2d, phi2d].';
qdd  = [x1dd y1dd phi1dd, x2dd, y2dd, phi2dd].';

%% Mapping
% center of mass positions as function of the generalized coordinates

% body 1
% - 

% body 2
% -

% holonomic constraints
C1 = x1 + La * cos(phi1) - x2;
C2 = y1 + La * sin(phi1) - y2;

% non-holonomic constraints
S1 = - x1d * sin(phi1) + y1d * cos(phi1) - Lb * phi1d; % tire back
S2 = - x2d * sin(phi2) + y2d * cos(phi2); % tire front

% mass
M = [m1, m1, I1, m2, m2, I2];
M = diag(M);

%% Differentiate constraints
% holonomic
C = [C1; C2];
Cq  = simplify(jacobian(C,q));
Cd = Cq * qd; % this is dC/dt=dC/dx*xd
C2 = simplify(jacobian(Cd,q) * qd);

% non holonomic
S = [S1; S2];
Sq  = simplify(jacobian(S,qd));
Sd = Sq * qd; % this is dC/dt=dC/dx*xd
S2 = simplify(jacobian(Sd,q) * qd);

%% EOM in matrix form
% forces
F = [cos(phi1)*force, sin(phi1)*force, 0, 0, 0, torque].';

% zero matrix
O = zeros(2,2);

LHS = [M, Cq.', Sq.'; Cq, O, O; Sq, O, O];

% export to latex
% disp(latex_pretify(LHS));

% Compute answer
result = LHS \ [F; -C2; -S2];
qdd = simplify(result(1:6));
lambda = simplify(result(7:length(result)));

%% Save symbolic derivation to script file.
% Use Diary function, save the symbolicly derived functions to file. 

% qdd
if exist('symb\symb_qdd.m', 'file')
    ! del symb\symb_qdd.m
end
diary symb\symb_qdd.m
    disp('qdd = ['), disp(qdd), disp('];');
diary off

% lambda
if exist('symb\symb_lambda.m', 'file')
    ! del symb\symb_lambda.m
end
diary symb\symb_lambda.m
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
    disp('Cq = ['), disp(Cq), disp('];');
diary off