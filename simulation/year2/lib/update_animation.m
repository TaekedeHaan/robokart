function l_current = update_animation(f, l_previous, y)

par = load_param();
[La, Lb, L, m1, m2, I1, I2] = unfold_param(par);

% store loactions
A = [y(1) - Lb * cos(y(3)), y(2) - Lb * sin(y(3))];
B1 = [y(1) + La * cos(y(3)), y(2) + La * sin(y(3))];
B2 = [y(4), y(5)];
C = [y(4), y(5)];

if ishandle(f)
    figure(f)
    hold on
    
    color = [1, 0.2, 0.2];
    
    delete(l_previous)
    l_current(1) = plot([A( 1), B1( 1)], [A( 2), B1( 2)], 'Color',color);
    l_current(2) = plot([B2( 1), C( 1)], [B2( 2), C( 2)], 'Color',color);
    
    tic
    drawnow
end

end