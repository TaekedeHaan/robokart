%% Load
load('test_05.mat')
load('lp_filter.mat')

%% Get data
t = S.t;
idx_end = find(t>0,1,'last');

t = t(1:3:idx_end);
xp = S.xp(1:3:idx_end);
yp = S.yp(1:3:idx_end);
xm = S.xm(1:3:idx_end);

%% Apply filter

dt = lp_fil.Ts;

t_real = t/1000;
t_sample = (0:dt:((numel(t)-1)*dt)).';

% plot(t_real), hold on
% plot(t_sample)

xpf = lsim(lp_fil,xp,t_sample);
ypf = lsim(lp_fil,yp,t_sample);
xmf = lsim(lp_fil,xm,t_sample);

figure(1)
plot(t_real,xpf), hold on
plot(t_real,ypf)
plot(t_real,xmf)

ax = gca();

playback = plot([0 0],ax.YLim);

%% Build map

figure(2)
h = plotMap([],xpf(1),ypf(1),xmf(1)); hold on
grid on

plotRadius = 1000;
r = plotRadius*computeDir(xpf(1),ypf(1),xmf(1));
q = quiver(0,0,r(1), r(2));
q.LineWidth = 1;

distMax = 3000;

xlim([-distMax distMax])
ylim([-distMax distMax])

tmax = max(t_real);

tic
while toc < tmax
    
    idx_c = find(t_real < toc,1,'last');
    h = plotMap(h,xpf(idx_c),ypf(idx_c),xmf(idx_c));
    
    tc = t_real(idx_c);
    playback.XData = [tc tc];
    
    r = plotRadius*computeDir(xpf(idx_c),ypf(idx_c),xmf(idx_c));
	q.UData = r(1);
    q.VData = r(2);
    
end


%% Helper functions
function vec = computeDir(xp,yp,xm)

distmin = 200;
vmax = 1;
cdist = -log(0.5)/distmin;

ycomp = max(-vmax,vmax - 2*vmax*exp(-cdist*(yp-distmin)));

xpcomp = max(-vmax,vmax - 2*vmax*exp(-cdist*(xp-distmin)));
xmcomp = max(-vmax,vmax - 2*vmax*exp(-cdist*(xm-distmin)));

xcomp = mean([xpcomp,-xmcomp]);

vec = [xcomp, ycomp];

end

function h = plotMap(h,xp,yp,xm)

xl = [-xm, 0, xp];
yl = [0, yp, 0];


if isempty(h)
    h = plot(xl,yl,'k*-');
else
    h.XData = xl;
    h.YData = yl;
end
drawnow
end