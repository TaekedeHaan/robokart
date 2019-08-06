function [Fa, Fb] = get_tire_force(t, y, force, torque)
q = y(1:4);
x = get_x(q);
[xCoM, yCoM, phi, xa, ya, phia, xb, yb, phib] = unfold_x(x);


lambda = get_forces_system(t, y, force, torque);

FAfric = lambda(1)*[sin(phia), -cos(phia)];
FAThrust = 1/2*force*[cos(phia), sin(phia)];
Fa = FAfric + FAThrust;

% forces back tire
FBFric = lambda(2)*[sin(phib), -cos(phib)];
FBThrust = 1/2*force*[cos(phib), sin(phib)];
Fb = FBFric + FBThrust;

end