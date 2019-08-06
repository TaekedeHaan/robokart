function xd = get_xd(q, qd)
% get parameters
par = load_param();
[La, Lb, L, ma, mb, m, Ia, Ib, I, bLin, bRot] = unfold_param(par);
x = q(1);
y = q(2);
phi = q(3);
alpha = q(4);
xd = qd(1);
yd = qd(2);
phid = qd(3);
alphad = qd(4);
xd = [
[ xd, yd, phid, xd - La*phid*sin(phi), yd + La*phid*cos(phi), alphad + phid, xd + Lb*phid*sin(phi), yd - Lb*phid*cos(phi), phid]
 
];
