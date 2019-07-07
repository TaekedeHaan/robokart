function [x, y, phi, alpha, xd, yd, phid, alphad] = unfold_y(state)
    % Author: Taeke de Haan
    % Date: 30-05-2018
    x      = state(1);
    y      = state(2);
    phi    = state(3); 
    alpha  = state(4);
    
    xd      = state(5);
    yd      = state(6);
    phid    = state(7); 
    alphad  = state(8);
end