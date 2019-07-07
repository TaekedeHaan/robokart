function [yd, lambda] = compute_system(y, par, force, torque)
    % Author: Taeke de Haan
    % Date: 30-05-2018
% unpack
qd = y(7:end);

% unfold parameters
[La, Lb, L, m1, m2, I1, I2] = unfold_param(par);

%unpack y 
[x1, y1, phi1, xa, ya, phia, xb, yb, phib, x1d, y1d, phi1d, xad, yad, phiad, xbd, ybd, phibd] = unfold(y)

symb_qdd;
symb_lambda;

yd = [qd, qdd'];
%ydot = [phi1d; phi2d; qdd]; 
end