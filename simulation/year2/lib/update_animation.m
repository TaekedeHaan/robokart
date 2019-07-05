function l_current = update_animation(f, l_previous, y, lamba, force)

scale = 5;

par = load_param();
[La, Lb, L, m1, m2, I1, I2] = unfold_param(par);

% store loactions
B1 = [y(1) - Lb*cos(y(3)), y(2) - Lb*sin(y(3))];
B2 = [y(1) + La*cos(y(3)), y(2) + La*sin(y(3))];
A1 = [y(4) - scale*par.r*cos(y(6)), y(5) -  scale*par.r*sin(y(6)) ];
A2 = [y(4) + scale*par.r*cos(y(6)), y(5) +  scale*par.r*sin(y(6)) ];

% forces
B = [y(1), y(2)];
FB = [-lamba(3)*sin(y(3)), lamba(3)*cos(y(3))]/50;

% forces
A = [y(4), y(5)];
FA1 = [-lamba(4)*sin(y(6)), lamba(4)*cos(y(6))]/50;




if ishandle(f)
    figure(f)
    hold on
    
    % normalized force, for color
    FAn = min(abs(lamba(3))/50, 1); 
    FBn = min(abs(lamba(4))/50, 1); 
    colorA = [FAn, 1 - FAn, 1 - FAn]; % [r, g, b]
    colorB = [FBn, 1 - FBn, 1 - FBn]; % [r, g, b]
    
    
    
    delete(l_previous)
    l_current(1) = plot([B1(1), B2(1)], [B1(2), B2(2)], 'Color','r');
    l_current(2) = plot([A1(1), A2(1)], [A1(2), A2(2)], 'Color','b');
    l_current(3) = plot([B(1), B(1) + FB(1)], [B(2), B(2) + FB(2)], 'Color',colorA, 'LineWidth', 1);
    l_current(4) = plot([A(1), A(1) + FA1(1)], [A(2), A(2) + FA1(2)], 'Color',colorB, 'LineWidth', 1);
    drawnow
end

end