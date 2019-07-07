function [La, Lb, L, ma, mb, m, Ia, Ib, I] = unfold_param(par)
% Author: Taeke de Haan
% Date: 30-05-2018
La = par.La;
Lb = par.Lb;
L = par.L;

m = par.m;
ma = par.ma;
mb = par.mb;

Ia = par.Ia;
Ib = par.Ib;
I = par.I;
end