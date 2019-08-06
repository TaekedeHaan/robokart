function x = get_x(q)
% get parameters
par = load_param();
[La, Lb, L, ma, mb, m, Ia, Ib, I, b] = unfold_param(par);
x = q(1);
y = q(2);
phi = q(3);
alpha = q(4);
x = [
               x
               y
             phi
 x + La*cos(phi)
 y + La*sin(phi)
     alpha + phi
 x - Lb*cos(phi)
 y - Lb*sin(phi)
             phi
 
];
