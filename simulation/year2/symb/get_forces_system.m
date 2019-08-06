function lambda = get_forces_system(t, y, force, torque);
par = load_param();
[La, Lb, L, ma, mb, m, Ia, Ib, I, bLin, bRot] = unfold_param(par);
%unpack y
[x, y, phi, alpha, xd, yd, phid, alphad] = unfold_y(y);
lambda = [
                                                                                                                                                                                                                                                                     -(2*I*force*sin(alpha) + 2*Ib*force*sin(alpha) + I*bLin*yd*cos(alpha + phi) + Ib*bLin*yd*cos(alpha + phi) - I*bLin*xd*sin(alpha + phi) - Ib*bLin*xd*sin(alpha + phi) + 2*La*m*torque*cos(alpha) + 2*Lb*m*torque*cos(alpha) + 2*La*ma*torque*cos(alpha) + 2*La*mb*torque*cos(alpha) + 2*Lb*ma*torque*cos(alpha) + 2*Lb*mb*torque*cos(alpha) - I*bLin*yd*cos(alpha - phi) - Ib*bLin*yd*cos(alpha - phi) - I*bLin*xd*sin(alpha - phi) - Ib*bLin*xd*sin(alpha - phi) + 2*Lb^2*force*m*sin(alpha) + 2*La^2*force*ma*sin(alpha) + 2*Lb^2*force*ma*sin(alpha) + I*m*phid*yd*sin(alpha - phi) + Ib*m*phid*yd*sin(alpha - phi) + I*ma*phid*yd*sin(alpha - phi) + I*mb*phid*yd*sin(alpha - phi) + Ib*ma*phid*yd*sin(alpha - phi) + Ib*mb*phid*yd*sin(alpha - phi) + 2*Lb^2*alphad*m^2*xd*cos(alpha + phi) + 2*La^2*alphad*ma^2*xd*cos(alpha + phi) + 2*Lb^2*alphad*ma^2*xd*cos(alpha + phi) - La^2*bLin*ma*yd*cos(alpha - phi) + Lb^2*bLin*mb*yd*cos(alpha - phi) + 2*Lb^2*m^2*phid*xd*cos(alpha + phi) + 2*La^2*ma^2*phid*xd*cos(alpha + phi) + 2*Lb^2*ma^2*phid*xd*cos(alpha + phi) + 2*Lb^2*alphad*m^2*yd*sin(alpha + phi) - La^2*bLin*ma*xd*sin(alpha - phi) + Lb^2*bLin*mb*xd*sin(alpha - phi) + 2*La^2*alphad*ma^2*yd*sin(alpha + phi) + 2*Lb^2*alphad*ma^2*yd*sin(alpha + phi) + 2*Lb^2*m^2*phid*yd*sin(alpha + phi) + 2*La^2*ma^2*phid*yd*sin(alpha + phi) + 2*Lb^2*ma^2*phid*yd*sin(alpha + phi) + 2*La^3*alphad*ma^2*phid*sin(alpha) + 2*I*alphad*m*xd*cos(alpha + phi) + 2*Ib*alphad*m*xd*cos(alpha + phi) + 2*I*alphad*ma*xd*cos(alpha + phi) + 2*I*alphad*mb*xd*cos(alpha + phi) + 2*Ib*alphad*ma*xd*cos(alpha + phi) + 2*Ib*alphad*mb*xd*cos(alpha + phi) + I*m*phid*xd*cos(alpha + phi) + Ib*m*phid*xd*cos(alpha + phi) + I*ma*phid*xd*cos(alpha + phi) + I*mb*phid*xd*cos(alpha + phi) + Ib*ma*phid*xd*cos(alpha + phi) + Ib*mb*phid*xd*cos(alpha + phi) + 2*I*alphad*m*yd*sin(alpha + phi) + 2*Ib*alphad*m*yd*sin(alpha + phi) + 2*I*alphad*ma*yd*sin(alpha + phi) + 2*I*alphad*mb*yd*sin(alpha + phi) + 2*Ib*alphad*ma*yd*sin(alpha + phi) + 2*Ib*alphad*mb*yd*sin(alpha + phi) + 4*La*Lb*force*ma*sin(alpha) - 2*La*alphad*bRot*m*cos(alpha) - 2*Lb*alphad*bRot*m*cos(alpha) - 2*La*alphad*bRot*ma*cos(alpha) - 2*La*alphad*bRot*mb*cos(alpha) - 2*Lb*alphad*bRot*ma*cos(alpha) - 2*Lb*alphad*bRot*mb*cos(alpha) + I*m*phid*yd*sin(alpha + phi) + Ib*m*phid*yd*sin(alpha + phi) + I*ma*phid*yd*sin(alpha + phi) + I*mb*phid*yd*sin(alpha + phi) + Ib*ma*phid*yd*sin(alpha + phi) + Ib*mb*phid*yd*sin(alpha + phi) - 2*La*bRot*m*phid*cos(alpha) - 2*Lb*bRot*m*phid*cos(alpha) - 2*La*bRot*ma*phid*cos(alpha) - 2*La*bRot*mb*phid*cos(alpha) - 2*Lb*bRot*ma*phid*cos(alpha) - 2*Lb*bRot*mb*phid*cos(alpha) + 2*Lb^2*bLin*m*yd*cos(alpha + phi) + La^2*bLin*ma*yd*cos(alpha + phi) + 2*Lb^2*bLin*ma*yd*cos(alpha + phi) + Lb^2*bLin*mb*yd*cos(alpha + phi) - I*m*phid*xd*cos(alpha - phi) - Ib*m*phid*xd*cos(alpha - phi) - I*ma*phid*xd*cos(alpha - phi) - I*mb*phid*xd*cos(alpha - phi) - Ib*ma*phid*xd*cos(alpha - phi) - Ib*mb*phid*xd*cos(alpha - phi) - 2*Lb^2*bLin*m*xd*sin(alpha + phi) - La^2*bLin*ma*xd*sin(alpha + phi) - 2*Lb^2*bLin*ma*xd*sin(alpha + phi) - Lb^2*bLin*mb*xd*sin(alpha + phi) + 2*I*La*alphad*m*phid*sin(alpha) + 2*Ib*La*alphad*m*phid*sin(alpha) + 2*I*La*alphad*ma*phid*sin(alpha) + 2*I*La*alphad*mb*phid*sin(alpha) + 2*Ib*La*alphad*ma*phid*sin(alpha) + 2*Ib*La*alphad*mb*phid*sin(alpha) + 4*La*Lb*alphad*ma^2*xd*cos(alpha + phi) + La*Lb*bLin*m*yd*cos(alpha - phi) - La*Lb*bLin*ma*yd*cos(alpha - phi) + La*Lb*bLin*mb*yd*cos(alpha - phi) + La*Lb*m^2*phid*xd*cos(alpha + phi) + 4*La*Lb*ma^2*phid*xd*cos(alpha + phi) + La*Lb*bLin*m*xd*sin(alpha - phi) - La*Lb*bLin*ma*xd*sin(alpha - phi) + La*Lb*bLin*mb*xd*sin(alpha - phi) + 4*La*Lb*alphad*ma^2*yd*sin(alpha + phi) + La*Lb*m^2*phid*yd*sin(alpha + phi) + 4*La*Lb*ma^2*phid*yd*sin(alpha + phi) + 2*La^2*alphad*m*ma*xd*cos(alpha + phi) + 4*Lb^2*alphad*m*ma*xd*cos(alpha + phi) + 2*Lb^2*alphad*m*mb*xd*cos(alpha + phi) + 2*La^2*alphad*ma*mb*xd*cos(alpha + phi) + 2*Lb^2*alphad*ma*mb*xd*cos(alpha + phi) + 2*La^2*m*ma*phid*xd*cos(alpha + phi) + 4*Lb^2*m*ma*phid*xd*cos(alpha + phi) + 2*Lb^2*m*mb*phid*xd*cos(alpha + phi) + 2*La^2*ma*mb*phid*xd*cos(alpha + phi) + 2*Lb^2*ma*mb*phid*xd*cos(alpha + phi) + 2*La^2*alphad*m*ma*yd*sin(alpha + phi) + 4*Lb^2*alphad*m*ma*yd*sin(alpha + phi) + 2*Lb^2*alphad*m*mb*yd*sin(alpha + phi) + 2*La^2*alphad*ma*mb*yd*sin(alpha + phi) + 2*Lb^2*alphad*ma*mb*yd*sin(alpha + phi) + 2*La^2*m*ma*phid*yd*sin(alpha + phi) + 4*Lb^2*m*ma*phid*yd*sin(alpha + phi) + 2*Lb^2*m*mb*phid*yd*sin(alpha + phi) + 2*La^2*ma*mb*phid*yd*sin(alpha + phi) + 2*Lb^2*ma*mb*phid*yd*sin(alpha + phi) + 2*La^3*alphad*m*ma*phid*sin(alpha) + 2*La^3*alphad*ma*mb*phid*sin(alpha) + La*Lb*m^2*phid*xd*cos(alpha - phi) - La*Lb*m^2*phid*yd*sin(alpha - phi) + 2*La*Lb^2*alphad*m^2*phid*sin(alpha) + 2*La*Lb^2*alphad*ma^2*phid*sin(alpha) + 4*La^2*Lb*alphad*ma^2*phid*sin(alpha) + La*Lb*bLin*m*yd*cos(alpha + phi) + 3*La*Lb*bLin*ma*yd*cos(alpha + phi) + La*Lb*bLin*mb*yd*cos(alpha + phi) - La*Lb*bLin*m*xd*sin(alpha + phi) - 3*La*Lb*bLin*ma*xd*sin(alpha + phi) - La*Lb*bLin*mb*xd*sin(alpha + phi) + 4*La*Lb*alphad*m*ma*xd*cos(alpha + phi) + 4*La*Lb*alphad*ma*mb*xd*cos(alpha + phi) + 5*La*Lb*m*ma*phid*xd*cos(alpha + phi) + La*Lb*m*mb*phid*xd*cos(alpha + phi) + 4*La*Lb*ma*mb*phid*xd*cos(alpha + phi) + 4*La*Lb*alphad*m*ma*yd*sin(alpha + phi) + 4*La*Lb*alphad*ma*mb*yd*sin(alpha + phi) + 5*La*Lb*m*ma*phid*yd*sin(alpha + phi) + La*Lb*m*mb*phid*yd*sin(alpha + phi) + 4*La*Lb*ma*mb*phid*yd*sin(alpha + phi) + La*Lb*m*ma*phid*xd*cos(alpha - phi) + La*Lb*m*mb*phid*xd*cos(alpha - phi) - La*Lb*m*ma*phid*yd*sin(alpha - phi) - La*Lb*m*mb*phid*yd*sin(alpha - phi) + 4*La*Lb^2*alphad*m*ma*phid*sin(alpha) + 4*La^2*Lb*alphad*m*ma*phid*sin(alpha) + 2*La*Lb^2*alphad*m*mb*phid*sin(alpha) + 2*La*Lb^2*alphad*ma*mb*phid*sin(alpha) + 4*La^2*Lb*alphad*ma*mb*phid*sin(alpha))/(I + Ib + La^2*m + 2*Lb^2*m + 2*La^2*ma + La^2*mb + 2*Lb^2*ma + Lb^2*mb - I*cos(2*alpha) - Ib*cos(2*alpha) + 2*La*Lb*m + 4*La*Lb*ma + 2*La*Lb*mb + La^2*m*cos(2*alpha) + La^2*mb*cos(2*alpha) + Lb^2*mb*cos(2*alpha) + 2*La*Lb*m*cos(2*alpha) + 2*La*Lb*mb*cos(2*alpha))
 -(2*I*bLin*yd*cos(phi) - 2*Ib*force*sin(2*alpha) - 2*La*m*torque - 4*Lb*m*torque - 4*La*ma*torque - 2*La*mb*torque - 4*Lb*ma*torque - 2*Lb*mb*torque - 2*I*force*sin(2*alpha) + 2*Ib*bLin*yd*cos(phi) - 2*I*bLin*xd*sin(phi) - 2*Ib*bLin*xd*sin(phi) - 2*I*bLin*yd*cos(2*alpha + phi) - 2*Ib*bLin*yd*cos(2*alpha + phi) + 2*I*bLin*xd*sin(2*alpha + phi) + 2*Ib*bLin*xd*sin(2*alpha + phi) - 2*La*m*torque*cos(2*alpha) - 2*La*mb*torque*cos(2*alpha) - 2*Lb*mb*torque*cos(2*alpha) + 2*La*alphad*bRot*m + 4*Lb*alphad*bRot*m + 4*La*alphad*bRot*ma + 2*La*alphad*bRot*mb + 4*Lb*alphad*bRot*ma + 2*Lb*alphad*bRot*mb + 2*La*bRot*m*phid + 4*Lb*bRot*m*phid + 4*La*bRot*ma*phid + 2*La*bRot*mb*phid + 4*Lb*bRot*ma*phid + 2*Lb*bRot*mb*phid + 2*La*alphad*bRot*m*cos(2*alpha) + 2*La*alphad*bRot*mb*cos(2*alpha) + 2*Lb*alphad*bRot*mb*cos(2*alpha) - 2*I*m*phid*yd*sin(2*alpha + phi) - 2*Ib*m*phid*yd*sin(2*alpha + phi) - 2*I*ma*phid*yd*sin(2*alpha + phi) - 2*I*mb*phid*yd*sin(2*alpha + phi) - 2*Ib*ma*phid*yd*sin(2*alpha + phi) - 2*Ib*mb*phid*yd*sin(2*alpha + phi) + 2*La*bRot*m*phid*cos(2*alpha) + 2*La*bRot*mb*phid*cos(2*alpha) + 2*Lb*bRot*mb*phid*cos(2*alpha) + La^2*m^2*phid*xd*cos(2*alpha - phi) + La^2*mb^2*phid*xd*cos(2*alpha - phi) + Lb^2*mb^2*phid*xd*cos(2*alpha - phi) - La^2*m^2*phid*yd*sin(2*alpha - phi) - La^2*mb^2*phid*yd*sin(2*alpha - phi) - Lb^2*mb^2*phid*yd*sin(2*alpha - phi) + 2*La^2*bLin*m*yd*cos(phi) + 4*La^2*bLin*ma*yd*cos(phi) + 2*La^2*bLin*mb*yd*cos(phi) - 2*La^2*bLin*m*xd*sin(phi) - 4*La^2*bLin*ma*xd*sin(phi) - 2*La^2*bLin*mb*xd*sin(phi) + La^2*bLin*m*yd*cos(2*alpha + phi) + La^2*bLin*mb*yd*cos(2*alpha + phi) - La^2*bLin*m*xd*sin(2*alpha + phi) - La^2*bLin*mb*xd*sin(2*alpha + phi) + 2*La^2*m^2*phid*xd*cos(phi) + 2*La^2*mb^2*phid*xd*cos(phi) + 2*Lb^2*mb^2*phid*xd*cos(phi) + 2*La^2*m^2*phid*yd*sin(phi) + 2*La^2*mb^2*phid*yd*sin(phi) + 2*Lb^2*mb^2*phid*yd*sin(phi) + La^2*bLin*m*yd*cos(2*alpha - phi) + La^2*bLin*mb*yd*cos(2*alpha - phi) + La^2*m^2*phid*xd*cos(2*alpha + phi) + La^2*mb^2*phid*xd*cos(2*alpha + phi) + Lb^2*mb^2*phid*xd*cos(2*alpha + phi) + La^2*bLin*m*xd*sin(2*alpha - phi) + La^2*bLin*mb*xd*sin(2*alpha - phi) - 2*I*alphad*m*xd*cos(phi) - 2*Ib*alphad*m*xd*cos(phi) - 2*I*alphad*ma*xd*cos(phi) - 2*I*alphad*mb*xd*cos(phi) - 2*Ib*alphad*ma*xd*cos(phi) - 2*Ib*alphad*mb*xd*cos(phi) + La^2*m^2*phid*yd*sin(2*alpha + phi) + La^2*mb^2*phid*yd*sin(2*alpha + phi) + Lb^2*mb^2*phid*yd*sin(2*alpha + phi) + 2*I*m*phid*xd*cos(phi) + 2*Ib*m*phid*xd*cos(phi) + 2*I*ma*phid*xd*cos(phi) + 2*I*mb*phid*xd*cos(phi) + 2*Ib*ma*phid*xd*cos(phi) + 2*Ib*mb*phid*xd*cos(phi) - 2*I*alphad*m*yd*sin(phi) - 2*Ib*alphad*m*yd*sin(phi) - 2*I*alphad*ma*yd*sin(phi) - 2*I*alphad*mb*yd*sin(phi) - 2*Ib*alphad*ma*yd*sin(phi) - 2*Ib*alphad*mb*yd*sin(phi) + 2*I*m*phid*yd*sin(phi) + 2*Ib*m*phid*yd*sin(phi) - 2*I*alphad*m*xd*cos(2*alpha + phi) - 2*Ib*alphad*m*xd*cos(2*alpha + phi) + 2*I*ma*phid*yd*sin(phi) + 2*I*mb*phid*yd*sin(phi) + 2*Ib*ma*phid*yd*sin(phi) + 2*Ib*mb*phid*yd*sin(phi) - 2*I*alphad*ma*xd*cos(2*alpha + phi) - 2*I*alphad*mb*xd*cos(2*alpha + phi) - 2*Ib*alphad*ma*xd*cos(2*alpha + phi) - 2*Ib*alphad*mb*xd*cos(2*alpha + phi) - 2*I*m*phid*xd*cos(2*alpha + phi) - 2*Ib*m*phid*xd*cos(2*alpha + phi) - 2*I*ma*phid*xd*cos(2*alpha + phi) - 2*I*mb*phid*xd*cos(2*alpha + phi) - 2*Ib*ma*phid*xd*cos(2*alpha + phi) - 2*Ib*mb*phid*xd*cos(2*alpha + phi) - 2*I*alphad*m*yd*sin(2*alpha + phi) - 2*Ib*alphad*m*yd*sin(2*alpha + phi) - 2*I*alphad*ma*yd*sin(2*alpha + phi) - 2*I*alphad*mb*yd*sin(2*alpha + phi) - 2*Ib*alphad*ma*yd*sin(2*alpha + phi) - 2*Ib*alphad*mb*yd*sin(2*alpha + phi) + 2*La*Lb*force*m*sin(2*alpha) + 2*La*Lb*mb^2*phid*xd*cos(2*alpha - phi) - 2*La*Lb*mb^2*phid*yd*sin(2*alpha - phi) + 2*La*Lb*bLin*m*yd*cos(phi) + 4*La*Lb*bLin*ma*yd*cos(phi) + 2*La*Lb*bLin*mb*yd*cos(phi) + 2*La^2*m*mb*phid*xd*cos(2*alpha - phi) - 2*La*Lb*bLin*m*xd*sin(phi) - 4*La*Lb*bLin*ma*xd*sin(phi) - 2*La*Lb*bLin*mb*xd*sin(phi) + 2*La^2*Lb*alphad*m^2*phid*sin(2*alpha) - 2*La^2*m*mb*phid*yd*sin(2*alpha - phi) + 2*La*Lb*bLin*m*yd*cos(2*alpha + phi) + La*Lb*bLin*mb*yd*cos(2*alpha + phi) - 2*La*Lb*bLin*m*xd*sin(2*alpha + phi) - La*Lb*bLin*mb*xd*sin(2*alpha + phi) - 2*I*La*alphad*m*phid*sin(2*alpha) - 2*Ib*La*alphad*m*phid*sin(2*alpha) - 2*I*La*alphad*ma*phid*sin(2*alpha) - 2*I*La*alphad*mb*phid*sin(2*alpha) - 2*Ib*La*alphad*ma*phid*sin(2*alpha) - 2*Ib*La*alphad*mb*phid*sin(2*alpha) + 2*La*Lb*alphad*m^2*xd*cos(phi) + 2*La*Lb*m^2*phid*xd*cos(phi) + 4*La*Lb*mb^2*phid*xd*cos(phi) + 2*La*Lb*alphad*m^2*yd*sin(phi) + 2*La*Lb*m^2*phid*yd*sin(phi) + 2*La*Lb*alphad*m^2*xd*cos(2*alpha + phi) + 4*La*Lb*mb^2*phid*yd*sin(phi) + La*Lb*bLin*mb*yd*cos(2*alpha - phi) + 2*La*Lb*m^2*phid*xd*cos(2*alpha + phi) + 2*La*Lb*mb^2*phid*xd*cos(2*alpha + phi) + 4*La^2*m*ma*phid*xd*cos(phi) + 4*La^2*m*mb*phid*xd*cos(phi) + 4*Lb^2*m*mb*phid*xd*cos(phi) + 4*La^2*ma*mb*phid*xd*cos(phi) + 4*Lb^2*ma*mb*phid*xd*cos(phi) + 2*La*Lb*alphad*m^2*yd*sin(2*alpha + phi) + La*Lb*bLin*mb*xd*sin(2*alpha - phi) + 2*La*Lb*m^2*phid*yd*sin(2*alpha + phi) + 2*La*Lb*mb^2*phid*yd*sin(2*alpha + phi) + 4*La^2*m*ma*phid*yd*sin(phi) + 4*La^2*m*mb*phid*yd*sin(phi) + 4*Lb^2*m*mb*phid*yd*sin(phi) + 4*La^2*ma*mb*phid*yd*sin(phi) + 4*Lb^2*ma*mb*phid*yd*sin(phi) + 2*La^2*m*mb*phid*xd*cos(2*alpha + phi) + 2*La^2*m*mb*phid*yd*sin(2*alpha + phi) + 2*La*Lb*m*mb*phid*xd*cos(2*alpha - phi) - 2*La*Lb*m*mb*phid*yd*sin(2*alpha - phi) + 2*La^2*Lb*alphad*m*ma*phid*sin(2*alpha) + 2*La^2*Lb*alphad*m*mb*phid*sin(2*alpha) + 2*La*Lb*alphad*m*ma*xd*cos(phi) + 2*La*Lb*alphad*m*mb*xd*cos(phi) + 2*La*Lb*m*ma*phid*xd*cos(phi) + 6*La*Lb*m*mb*phid*xd*cos(phi) + 8*La*Lb*ma*mb*phid*xd*cos(phi) + 2*La*Lb*alphad*m*ma*yd*sin(phi) + 2*La*Lb*alphad*m*mb*yd*sin(phi) + 2*La*Lb*m*ma*phid*yd*sin(phi) + 6*La*Lb*m*mb*phid*yd*sin(phi) + 2*La*Lb*alphad*m*ma*xd*cos(2*alpha + phi) + 2*La*Lb*alphad*m*mb*xd*cos(2*alpha + phi) + 8*La*Lb*ma*mb*phid*yd*sin(phi) + 2*La*Lb*m*ma*phid*xd*cos(2*alpha + phi) + 4*La*Lb*m*mb*phid*xd*cos(2*alpha + phi) + 2*La*Lb*alphad*m*ma*yd*sin(2*alpha + phi) + 2*La*Lb*alphad*m*mb*yd*sin(2*alpha + phi) + 2*La*Lb*m*ma*phid*yd*sin(2*alpha + phi) + 4*La*Lb*m*mb*phid*yd*sin(2*alpha + phi))/(4*(I + Ib + Lb^2*m + La^2*ma + Lb^2*ma - I*cos(alpha)^2 - Ib*cos(alpha)^2 + 2*La*Lb*ma + La^2*m*cos(alpha)^2 + La^2*mb*cos(alpha)^2 + Lb^2*mb*cos(alpha)^2 + 2*La*Lb*m*cos(alpha)^2 + 2*La*Lb*mb*cos(alpha)^2))
 
];
