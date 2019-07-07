function l_current = update_animation(f, l_previous, y, lamba, force)

scale = 5;

par = load_param();
[La, Lb, L, m1, m2, I1, I2] = unfold_param(par);

[x1, y1, phi1, xa, ya, phia, xb, yb, phib, ~, ~, ~, ~, ~, ~, ~, ~, ~] = unfold(y);

% body
CoM1 = [x1 - Lb*cos(phi1), y1 - Lb*sin(phi1)];
CoM2 = [x1 + La*cos(phi1), y1 + La*sin(phi1)];

% front wheel
A1 = [xa - scale*par.r*cos(phia), ya -  scale*par.r*sin(phia) ];
A2 = [xa + scale*par.r*cos(phia), ya +  scale*par.r*sin(phia) ];

% back wheel
B1 = [xb - Lb*cos(phib), yb - Lb*sin(phib)];
B2 = [xb + La*cos(phib), yb + La*sin(phib)];


% forces fron tire
A = [xa, ya];
FA1 = [-lamba(end)*sin(phia), lamba(end)*cos(phia)]/50;

% forces back tire
B = [xb, yb];
FB = [-lamba(end-1)*sin(phib), lamba(end-1)*cos(phib)]/50;



if ishandle(f)
figure(f)
hold on

% normalized force, for color
FAn = min(abs(lamba(3))/50, 1);
FBn = min(abs(lamba(4))/50, 1);
colorA = [FAn, 1 - FAn, 1 - FAn]; % [r, g, b]
colorB = [FBn, 1 - FBn, 1 - FBn]; % [r, g, b]

delete(l_previous)
l_current(1) = plot([CoM1(1), CoM2(1)], [CoM1(2), CoM2(2)], 'Color','r');
l_current(2) = plot([B1(1), B2(1)], [B1(2), B2(2)], 'Color','b');
l_current(3) = plot([A1(1), A2(1)], [A1(2), A2(2)], 'Color','b');
l_current(4) = plot([B(1), B(1) + FB(1)], [B(2), B(2) + FB(2)], 'Color',colorA, 'LineWidth', 1);
l_current(5) = plot([A(1), A(1) + FA1(1)], [A(2), A(2) + FA1(2)], 'Color',colorB, 'LineWidth', 1);
drawnow
end
end