function [torque, force] = get_control_action(steerRef, forceRef, y, par)

% compute control action
steerPosError = steerRef  - y(4);
steerVelError = - y(8);
torque = par.kpSteer*steerPosError + par.kdSteer*steerVelError;
    
% v = sqrt(y(5,i)^2 + y(6,i)^2); 
force = forceRef; %100 * (velocityRef  - v);    
    
% saturate if above capabilities
torque = max(min(torque, par.torqueMax), par.torqueMin);
force = max(min(force, par.forceMax), par.forceMin);

end