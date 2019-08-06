function l_current = update_animation(f, l_previous, y, lamba, force)

scale = 5;

par = load_param();
[La, Lb, L, ma, mb, m, Ia, Ib, I, b] = unfold_param(par)

q = y(1:4);
qd = y(5:8);
[x] = get_x(q);
[x1, y1, phi1, xa, ya, phia, xb, yb, phib] = unfold_x(x);

% body
CoM1 = [xa, ya];
CoM2 = [xb, yb];

% front wheel
A1 = [xa - scale*par.r*cos(phia), ya -  scale*par.r*sin(phia) ];
A2 = [xa + scale*par.r*cos(phia), ya +  scale*par.r*sin(phia) ];

% back wheel
B1 = [xb - Lb*cos(phib), yb - Lb*sin(phib)];
B2 = [xb + La*cos(phib), yb + La*sin(phib)];


% forces front tire
A = [xa, ya];
FAfric = -[-lamba(end-1)*sin(phia), lamba(end-1)*cos(phia)]/50;
FAThrust = [1/2*force*cos(phia), 1/2*force*sin(phia)]/50;

% forces back tire
B = [xb, yb];
FBFric = -[-lamba(end)*sin(phib), lamba(end)*cos(phib)]/50;
FBThrust = [1/2*force*cos(phib), 1/2*force*sin(phib)]/50;


if ishandle(f)
figure(f)
hold on

% normalized force, for color
FAn = min(abs(lamba(end-1))/50, 1);
FBn = min(abs(lamba(end))/50, 1);
colorA = [FAn, 1 - FAn, 1 - FAn]; % [r, g, b]
colorB = [FBn, 1 - FBn, 1 - FBn]; % [r, g, b]

delete(l_previous)
l_current(1) = plot([CoM1(1), CoM2(1)], [CoM1(2), CoM2(2)], 'Color','r');
l_current(2) = plot([B1(1), B2(1)], [B1(2), B2(2)], 'Color','b');
l_current(3) = plot([A1(1), A2(1)], [A1(2), A2(2)], 'Color','b');
l_current(4) = plot([B(1), B(1) + FBFric(1) + FBThrust(1)], [B(2), B(2) + FBFric(2) + FBThrust(2)], 'Color',colorA, 'LineWidth', 1);
l_current(5) = plot([A(1), A(1) + FAfric(1) + FAThrust(1)], [A(2), A(2) + FAfric(2) + FAThrust(2)], 'Color',colorB, 'LineWidth', 1);
drawnow
end
end