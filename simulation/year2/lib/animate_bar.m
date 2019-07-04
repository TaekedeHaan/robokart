function animate_bar(A, B1, B2, C, dt, fline, static)
    % Author: Taeke de Haan
    % Date: 30-05-2018

    P = size(A,1);

    p = figure('Position', [200,200,500,400]); %create figure
    grid on
    
    xlabel('x [m]')
    ylabel('y [m]')
    title('Animation')
    
    lowerLim = min([min(A(:,1)), min(A(:,2))]);
    upperLim = max([ max(C(:,1)),  max(C(:,2))]);
    axis([lowerLim, upperLim, lowerLim, upperLim])
    axis square
    
    plotStep = round((1/dt) / fline);

    for i = 1 : plotStep : size(A,1)
        if ishandle(p)
            figure(p)
            hold on
            
            if static             
                color = [1/2 * sin(i * 2 * pi / P) + 1/2, 1/2 * sin(i * 2 * pi / P + pi/4) + 1/2, 1/2 * sin(i * 2 * pi / P + pi/2) + 1/2];
            else
                color = [1, 0.2, 0.2];
            end
            
            if i ~= 1 && ~static
                 delete(l_current)
            end
            l_current(1) = plot([A(i, 1), B1(i, 1)], [A(i, 2), B1(i, 2)], 'Color',color);
            l_current(2) = plot([B2(i, 1), C(i, 1)], [B2(i, 2), C(i, 2)], 'Color',color);
            
            tic
            drawnow

            %wait some time, so the gait is played back at an accurate speed.
            while true
                if ~(toc < dt * plotStep)
                    break
                end
            end   
        else %If figure p is closed, stop animation.
            break
        end
    end
end