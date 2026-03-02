function [ nx, ny, nt ] = SimDiffRob( cx, cy, ct, it, v, w )
% SimDiffRob Computes next absolute pose for a robot given current
% state, delta time and velocities.

% INPUTS: 
% Current Pose
%   cx - current x [m]
%   cy - current y [m]
%   ct - current theta [rad]
% Command
%   it - delta time [sec]
%   v - trans vel [m/s]
%   w - rot vel [rad/sec]

% OUTPUTS: nx, ny, nt
% 

theta = w*it;
      
if abs(w)<1.0e-12
    y = v*it;
    x = 0;
else
    r = abs(v/w);

    if theta<=0.0 
        x = r * (1-cos(theta));
    else
        x = r * (cos(theta)-1); 
    end
    y = r * sin(abs(theta));
end
desp = sqrt(x^2 + y^2);

beta = atan2(x,y) - ct;

% Pose final
nx = cx + desp*sin(beta);
ny = cy + desp*cos(beta);
nt = ct + theta;
      
end
