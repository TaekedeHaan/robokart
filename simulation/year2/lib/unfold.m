function [x1, y1, phi1, xa, ya, phia, xb, yb, phib, x1d, y1d, phi1d, xad, yad, phiad, xbd, ybd, phibd] = unfold(y)
    % Author: Taeke de Haan
    % Date: 30-05-2018
    x1      = y(1);
    y1      = y(2);
    phi1    = y(3); 
    xa      = y(4);
    ya      = y(5);
    phia    = y(6);
    xb      = y(7);
    yb      = y(8);
    phib    = y(9);
    
    x1d     = y(10);
    y1d     = y(11);
    phi1d   = y(12);
    xad     = y(13);
    yad     = y(14);
    phiad   = y(15);
    xbd     = y(16);
    ybd     = y(17);
    phibd   = y(18);
end