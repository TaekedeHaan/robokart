%% Clear open COM ports
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

%%
% Create serial object
COMObj = serial('COM4','BAUD', 9600);
fopen(COMObj);
go = true;

xpos = 0;
ypos = 0;
xneg = 0;
tc = 0;

pos = [xpos, 0; 0, ypos; -xneg, 0];
xl = pos(:,1);
yl = pos(:,2);

% figure();

% line = plot(xl,yl,'k*-');
%
% distmax = 500;
%
% xlim([-distmax distmax])
% ylim([-distmax distmax])
%
% grid on
% drawnow

tmax = 40;

Nmax = 10000;

t = zeros(Nmax,1);
xp = zeros(Nmax,1);
yp = zeros(Nmax,1);
xm = zeros(Nmax,1);

% tstart = tic;
count = 1;

tic
while toc < tmax
    
    var = fscanf(COMObj);
    disp(var)
    
    IDs = {'X+','Y+','X-','t:'};
    
    if numel(var)>2
        idx = strcmpi(var(1:2),IDs);
    else
        idx = [];
    end
    
    if any(idx)
        ID = IDs{idx};
    end
    % ID = ID{:};
    
    switch ID
        case 'X+'
            xpos = str2double(var(3:end));
        case 'Y+'
            ypos = str2double(var(3:end));
        case 'X-'
            xneg = str2double(var(3:end));
        case 't:'
            tc = str2double(var(3:end));
    end
    
    % Append data if new time index was reached
    if count == 1 || tc > t(count-1)
        xp(count) = xpos;
        yp(count) = ypos;
        xm(count) = xneg;
        t(count)  = tc;
        
        count = count + 1;
    end
    
    %     vals = [xpos ypos xneg];
    %     vals(vals<0) = nan;
    %
    %     xpos = vals(1);
    %     ypos = vals(2);
    %     xneg = vals(3);
    %
    %     pos = [xpos, 0; 0, ypos; -xneg, 0];
    %     xl = pos(:,1);
    %     yl = pos(:,2);
    %
    %     line.XData = xl;
    %     line.YData = yl;
    %     drawnow
    
    %     dt = toc;
    %     pause(0.1 - dt)
    
    %     disp(pos)
end

%% Close communication
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end


%% Post processing
xp(xp<0) = nan;
yp(xp<0) = nan;
xm(xp<0) = nan;

idx_end = find(t,1,'last');

figure()
plot(t(1:idx_end),xp(1:idx_end)), hold on
plot(t(1:idx_end),yp(1:idx_end))
plot(t(1:idx_end),xm(1:idx_end))

legend('X+','Y+','X-')

xlabel('t [ms]')
ylabel('Distance [mm]')

%% Save
S.t = t;
S.xp = xp;
S.yp = yp;
S.xm = xm;

save('test_05','S')
