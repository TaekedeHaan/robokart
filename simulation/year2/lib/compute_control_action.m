function [force, torque] = compute_control_action(steerRef, velocityRef, y, par)

% compute control action
steerPosError = steerRef  - y(4);
steerVelError = - y(7);
torque = par.steepKp*steerPosError + par.steerKd*steerVelError;
v = sqrt(y(5)^2 + y(6)^2);
force = 100 * (velocityRef  - v);

% saturate if above capabilities
torque = max(min(torque, par.torqueMax), par.torqueMin);
force = max(min(force, par.forceMax), par.forceMin);

end