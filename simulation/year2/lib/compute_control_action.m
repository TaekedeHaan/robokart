function [force, torque] = compute_control_action(steerRef, velocityRef, y, par)

% compute control action
steerPosError = steerRef  - y(4);
steerVelError = 0 - y(8);

disp(y(8))
torque = par.steerKp*steerPosError + par.steerKd * steerVelError;
v = sqrt(y(5)^2 + y(6)^2);
force = 100 * (velocityRef  - v);

% saturate if above capabilities
torque = max(min(torque, par.torqueMax), par.torqueMin);
force = max(min(force, par.forceMax), par.forceMin);

end