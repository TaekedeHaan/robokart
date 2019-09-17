close all

A = 0.4;
B = 1 - A;
C = A;
D = B;

dt = 0.16; % s

lp_fil = ss(A,B,C,D,dt);

figure(1)
clf();
bode(lp_fil)

figure(2)
clf();
impulse(lp_fil)

co_freq = rad2deg(bandwidth(lp_fil))/360

%%

dist0 = 2000; % mm
vmax = 1*1000; % mm/s

tend = dist0/vmax;
t = 0:dt:tend;

dist = dist0 - vmax*t;

figure(3)
lsim(lp_fil,dist,t)

%% Test with actual data
load('test_05.mat')

N = nnz(S.yp);
idx_u = 1:3:N;

u = S.yp(idx_u);
t_real = S.t(idx_u)/1000;
t_sample = (0:dt:((numel(idx_u)-1)*dt)).';

figure
plot(t_real), hold on
plot(t_sample)

figure
lsim(lp_fil,u,t_sample)

%% Save
save('lp_filter','lp_fil')




