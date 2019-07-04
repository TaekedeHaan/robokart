function [x1, y1, phi1, x2, y2, phi2, x1d, y1d, phi1d, x2d, y2d, phi2d] = unfold(y)
    % Author: Taeke de Haan
    % Date: 30-05-2018
    x1      = y(1);
    y1      = y(2);
    phi1    = y(3); 
    x2      = y(4);
    y2      = y(5);
    phi2    = y(6);
    
    x1d     = y(7);
    y1d     = y(8);
    phi1d   = y(9);
    x2d     = y(10);
    y2d     = y(11);
    phi2d   = y(12);
end