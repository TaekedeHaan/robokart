function qdd = get_qdd_double_slip(y, F, par)
%unpack
[La, Lb, L, ma, mb, m, Ia, Ib, I, bLin, bRot, fFricMax] = unfold_param(par);
[xCoM, yCoM, phi, alpha, xCoMd, yCoMd, phid, alphad] = unfold_y(y);
[f1, f2, f3, f4, f5, f6, f7, f8 ,f9] = unfold_F(F);
qdd = [
                                                                                                                                                           (5*f1)/26 + (5*f4)/26 + (5*f7)/26 - (45*xCoMd)/52
                                                                                                                                                           (5*f2)/26 + (5*f5)/26 + (5*f8)/26 - (45*yCoMd)/52
               (53757760680212428125*alphad)/3551088306181636096 + (37500*f3)/1577 + (37500*f9)/1577 + (5625*f5*cos(phi))/1577 - (5625*f8*cos(phi))/1577 - (5625*f4*sin(phi))/1577 + (5625*f7*sin(phi))/1577
 18750*f6 - (37500*f3)/1577 - (84883504114055424009375*alphad)/7102176612363272192 - (37500*f9)/1577 - (5625*f5*cos(phi))/1577 + (5625*f8*cos(phi))/1577 + (5625*f4*sin(phi))/1577 - (5625*f7*sin(phi))/1577
 
];
