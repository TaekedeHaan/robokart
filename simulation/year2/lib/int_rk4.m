function y = int_rk4(y, dt, par, force, torque)
% Author: Taeke de Haan
% Date: 30-05-2018
%rk4 intergration
%   [y] = int_rk4(y, dt, t)

k1 = compute_system(y, par, force, torque);
k2 = compute_system(y + 1/2 * dt * k1, par, force, torque);
k3 = compute_system(y + 1/2 * dt * k2, par, force, torque);
k4 = compute_system(y + dt * k3, par, force, torque);
y = y + 1/6 * dt * (k1 + 2 * k2 + 2 * k3 + k4);
end