function f = init_animation(size, lim)

% P = size(A,1);

f = figure('Units','normalized','OuterPosition', size); %create figure
grid on

xlabel('x [m]')
ylabel('y [m]')
title('Animation')

axis([lim(1), lim(2), lim(1), lim(2)])
axis square
end