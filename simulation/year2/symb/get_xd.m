function xd = get_xd(q)
% get parameters
par = load_param();
[La, Lb, L, ma, mb, m, Ia, Ib, I, b] = unfold_param(par);
xd = qd(1);
yd = qd(2);
phid = qd(3);
alphad = qd(4);
xd = [
[ xd, yd, phid, xd - La*phid*sin(phi), yd + La*phid*cos(phi), alphad + phid, xd + Lb*phid*sin(phi), yd - Lb*phid*cos(phi), phid]
 
];
