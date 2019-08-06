function yd = get_acceleration_system(t, y, force, torque, par)
q = y(1:4);
qd = y(5:end);

% compute tire force vector and compute magnitude
[Fa, Fb] = get_tire_force(t, y, force, torque);
fa = sqrt(sum(Fa.^2));
fb = sqrt(sum(Fb.^2));

% limit de tire force
faLim = max(min(fa, par.fFricMax), -par.fFricMax); % lambda(1); % 
fbLim = max(min(fb, par.fFricMax), -par.fFricMax); % lambda(2); % 

% update the tire force vector
FaLim = faLim/fa*Fa;
FbLim = fbLim/fb*Fb;

% if we adjusted the tire force, 
if abs(fa - faLim) > 0.001 || abs(fb - fbLim) > 0.001
    slip = true;
else
    slip = false;
end
slip = false;

x = get_x(q);
xd = get_xd(q, qd);
[xCoM, yCoM, phi, xa, ya, phia, xb, yb, phib] = unfold_x(x);
[xCoMd, yCoMd, phid, xad, yad, phiad, xbd, ybd, phibd] = unfold_x(xd);

Fin = [0, 0, 0, FaLim(1), FaLim(2) , torque, FbLim(1), FbLim(2) , 0].';

% [vaPar, vaPerp] = compute_wheel_vel(xad, yad, phia);
% [vbPar, vbPerp] = compute_wheel_vel(xbd, ybd, phib);
% 

% disp(['A parrallel velocity: ', num2str(vaPar)]);
% disp(['A perpendicular velocity: ', num2str(vaPerp)]);


if slip
    % disp('slip')
    qdd = get_qdd_double_slip(y, Fin, par);
else
    % disp('no slip')
    qdd = get_qdd_no_slip(y, force, torque, par);
end

yd = [qd; qdd];
