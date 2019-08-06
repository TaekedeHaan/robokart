function x = get_x(q)
% get parameters
par = load_param();
[La, Lb, L, ma, mb, m, Ia, Ib, I, bLin, bRot, fFricMax] = unfold_param(par);
xCoM = q(1);
yCoM = q(2);
phi = q(3);
alpha = q(4);
x = [
                   xCoM
                   yCoM
                    phi
 xCoM + (3*cos(phi))/20
 yCoM + (3*sin(phi))/20
            alpha + phi
 xCoM - (3*cos(phi))/20
 yCoM - (3*sin(phi))/20
                    phi
 
];
