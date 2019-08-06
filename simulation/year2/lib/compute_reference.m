function [steerRef, velocityRef] = compute_reference(keyPress, steerRef, velocityRef, par)

if ~isempty(keyPress)
    switch keyPress
        case 'leftarrow'
            steerRef = par.steerRefMax; % [rad]
        case 'rightarrow'
            steerRef = par.steerRefMin; % [rad]
        case 'uparrow'
            velocityRef = par.vMax; % [m/s]
            steerRef = 0;
        case 'downarrow'
            velocityRef = 0; % [m/s]
        otherwise
            velocityRef = 0;
            steerRef = 0;
    end
else
    steerRef = 0;
    velocityRef = 0;
end

end