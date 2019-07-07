%% Generate equations
% Author: Taeke de Haan
% Date: 30-05-2018

clear all; close all; clc

%% Pendulum Constants (symbolic definition)
% length
syms L La Lb   real

% mass
syms m I ma mb Ia Ib   real

% load param
% [paramDim, paramMass] = load_param();
% [a, b, c, d] = unfold_param_dim(paramDim);
% [m1, m2, I1, I2] = unfold_param_mass(paramMass);

% other
syms g torque force           real

%% Generalized coordinates and their derivatives
syms x1 y1 phi1 xa ya phia xb yb phib real
syms x1d y1d phi1d xad yad phiad  xbd ybd phibd real
% syms x1dd y1dd phi1dd xadd yadd phiadd real

q   = [x1 y1 phi1, xa, ya, phia, xb, yb, phib].';
qd  = [x1d y1d phi1d, xad, yad, phiad, xbd, ybd, phibd].';

%% Mapping
% center of mass positions as function of the generalized coordinates

% body 1
% - 

% body 2
% -

% body 3

% holonomic constraints
C(1,1) = x1 + La * cos(phi1) - xa;
C(2,1) = y1 + La * sin(phi1) - ya;

C(3,1) = x1 - Lb * cos(phi1) - xb;
C(4,1) = y1 - Lb * sin(phi1) - yb;
C(5,1) = phi1 - phib;

% non-holonomic constraints
S1 = - xbd * sin(phib) + ybd * cos(phib); % tire back
S2 = - xad * sin(phia) + yad * cos(phia); % tire front

% mass
M = [m, m, I, ma, ma, Ia, mb, mb, Ib];
M = diag(M);

%% Differentiate constraints
% holonomic
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
F = [0, 0, 0, 1/2*cos(phi1)*force, 1/2*sin(phi1)*force, torque, 1/2*cos(phi1)*force, 1/2*sin(phi1)*force, 0].';

% zero matrix
O = zeros(length(C),length(C));

LHS = [ M,      Cq.',                       Sq.'; 
        Cq,     zeros(length(C),length(C)),	zeros(length(C),length(S));
        Sq,     zeros(length(S),length(C)),	zeros(length(S),length(S))];

% export to latex
% disp(latex_pretify(LHS));

% Compute answer
result = LHS \ [F; -C2; -S2];
qdd = simplify(result(1:length(q)));
lambda = simplify(result(length(q)+1:end));

%% Save symbolic derivation to script file.
% Use Diary function, save the symbolicly derived functions to file. 

% qdd
if exist('symb\get_acceleration_system.m', 'file')
    ! del symb\get_acceleration_system.m
end
diary symb\get_acceleration_system.m
    disp('function yd = get_acceleration_system(t, y, force, torque)')
    disp('qd = y(10:end);')

    disp('% get parameters')
    disp('par = load_param();');
    disp('[La, Lb, L, ma, mb, m, Ia, Ib, I] = unfold_param(par);');

    disp('%unpack y')
    disp('[x1, y1, phi1, xa, ya, phia, xb, yb, phib, x1d, y1d, phi1d, xad, yad, phiad, xbd, ybd, phibd] = unfold(y);');

    disp('qdd = ['), disp(qdd'), disp('];');
    disp('yd = [qd, qdd];');
diary off

% lambda
if exist('symb\get_forces_system.m', 'file')
    ! del symb\get_forces_system.m
end
diary symb\get_forces_system.m
    disp('function lambda = get_forces_system(t, y, force, torque)')
    
    disp('par = load_param();');
    disp('[La, Lb, L, ma, mb, m, Ia, Ib, I] = unfold_param(par);');
    
    
    disp('%unpack y')
    disp('[x1, y1, phi1, xa, ya, phia, xb, yb, phib, x1d, y1d, phi1d, xad, yad, phiad, xbd, ybd, phibd] = unfold(y);');

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