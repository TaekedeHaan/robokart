function xd = get_xd(q, qd)
% get parameters
par = load_param();
[La, Lb, L, ma, mb, m, Ia, Ib, I, bLin, bRot, fFricMax] = unfold_param(par);
xCoM = q(1);
yCoM = q(2);
phi = q(3);
alpha = q(4);
xCoMd = qd(1);
yCoMd = qd(2);
phid = qd(3);
alphad = qd(4);
xd = [
[ xCoMd, yCoMd, phid, xCoMd - (3*phid*sin(phi))/20, yCoMd + (3*phid*cos(phi))/20, alphad + phid, xCoMd + (3*phid*sin(phi))/20, yCoMd - (3*phid*cos(phi))/20, phid]
 
];
