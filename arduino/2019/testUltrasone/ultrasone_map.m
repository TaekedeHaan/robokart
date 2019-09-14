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

pos = [xpos, 0; 0, ypos; -xneg, 0];
xl = pos(:,1);
yl = pos(:,2);

figure();

line = plot(xl,yl,'k*-');

distmax = 500;

xlim([-distmax distmax])
ylim([-distmax distmax])

grid on
drawnow

while go
    tic
    % a= input('Press 1 to turn ON LED & 0 to turn OFF:');
    var = fscanf(COMObj);
    disp(var)
    
    IDs = {'X+','Y+','X-'};
    
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
    end
    
    vals = [xpos ypos xneg];
    vals(vals<0) = nan;
    
    xpos = vals(1);
    ypos = vals(2);
    xneg = vals(3);
    
    pos = [xpos, 0; 0, ypos; -xneg, 0];
    xl = pos(:,1);
    yl = pos(:,2);
    
    line.XData = xl;
    line.YData = yl;
    drawnow
    
    pause(0.1 - toc)
    
    disp(pos)
    
end