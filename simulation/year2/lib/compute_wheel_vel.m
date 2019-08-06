function [vPar, vPerp] = compute_wheel_vel(x, y, phi)


vPar = x*cos(phi) + y*sin(phi); 
vPerp = -x*sin(phi) + y*cos(phi); 

end