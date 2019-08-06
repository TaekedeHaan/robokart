function [steerRef, forceRef] = get_reference(keyPress, steerRef,  forceRef, par)

if ~isempty(keyPress)
    switch keyPress
        case 'leftarrow'
            steerRef = par.steerRefMax; % [rad]
        case 'rightarrow'
            steerRef = par.steerRefMin; % [rad]
        case 'uparrow'
            forceRef = par.forceMax; % [m/s]
            steerRef = par.steerRefDef;
        case 'downarrow'
            forceRef = 0; % [m/s]
        otherwise
            forceRef = 0;
            steerRef = par.steerRefDef;
    end
else
    steerRef = par.steerRefDef;
    forceRef = 0;
end

end