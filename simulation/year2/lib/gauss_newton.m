function y = gauss_newton(y, par)
% Author: Taeke de Haan
% Date: 30-05-2018
q = y(1:6);
qd = y(7:12);

iterate = 0;     % itterations
tol = 1e-12;    % tolrance
maxiterate = 10;  % maximum itterations

% unfold parameters
[La, Lb, L, m1, m2, I1, I2] = unfold_param(par);

% transfrom position
while true
    % unfold
    [x1, y1, phi1, x2, y2, phi2, x1d, y1d, phi1d, x2d, y2d phi2d] = unfold([q, qd]);
	
    symb_C; % C = C(q_n1 )
    symb_Cq;
    
    % Cq' * (Cq * Cq')^(-1) ?
    e = -C;                 % error
    delta = pinv(Cq) * e;   % Dq_n1 = -C,q^T(C,q*C,q^T)*C
    q = q + delta';         % q_n1 = q_n1 + Dq_n1
    
	iterate = iterate +1;
    
    if( max(abs(C)) < tol || iterate > maxiterate)
        break;
    end
end

[x1, y1, phi1, x2, y2, phi2, x1d, y1d, phi1d, x2d, y2d phi2d] = unfold([q, qd]);

% transform velocity
symb_Cq; 
symb_Sq;  % C = C(q_n1 )

ed = - ([Cq *  qd' ; Sq * qd']);       % 1 Cd = C,q*C*qd_n1
deltad = pinv([Cq; Sq]) * ed;     % 2 Dqd_n1 = -C,q^T(C,q*C,q^T)*Cd
qd = qd + deltad';      	% 3 set qd_n1 = qd_n1 + Dqd_n1

y = [q, qd];
end