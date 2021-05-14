%% Trajectory Plotting
% 
% PlotTrajectory calcucaltes the points in a parabolic arc as a pathway a
% projectile will follow.
%
% [ballTrajectory, velocity] = PlotTrajectory(robot,pose,target,pitch);

% OUTPUTS
% ballTrajectory = An array of (x,y,z) values of the trajectory 1x3x100.
% velocity = The velocity required to meet the requiremtns of the arc.

% INPUTS
% robot = The SimLink Robot Used to throw the robot
% pose = Q values that define the releasing point of the projectile
% target = The (x,y,z) co-ordinates of the target
% pitch = Angle of trajectory upon releasing
% plot = Toggles Plotting [0,1]


function [ballTrajectory, velocity] = PlotTrajectory(robot,pose,target,pitch,plot);

if plot > 0
    plot = 1;
else
    plot = 0;
end
%Fkine co-ordinates
EndPose = robot.model.fkine(pose);
Fx = EndPose(1,4);
Fy = EndPose(2,4);
Fz = EndPose(3,4);

%Target co-ordinates
Tx = target(1,1);
Ty = target(1,2);
Tz = target(1,3);

tol = 0.2; 

Xdif = Fx-Tx;
Ydif = Fy-Ty;

xyAngle = atand(Ydif/Xdif);                   %yaw angle
 if Ty < 0;
 xyAngle = xyAngle + 180;
 end
zAngle=pitch;                       %pitch angle
g=9.81;                             %gravity
hold on

if plot == 1        
    plot3(Fx,Fy,Fz,'g.', 'markers', 30);
    plot3(Tx,Ty,Tz,'r.', 'markers', 20);
end

for i = 0.01:0.005:3
v0 = i;    
    tges=(2*v0.*sin(zAngle.*(pi/180)))/g;
    t =linspace(0,tges);
    v_xy = v0.*cos(zAngle.*(pi/180));
    v_z = v0.*sin(zAngle.*(pi/180));
    sx = (v_xy.*t)/sind(xyAngle);
    sx = sx + Fx;
    sy = (v_xy.*t)/cosd(xyAngle);
    sy = sy + Fy;
    sz = v_z.*t-0.5*g.*t.^2;
    sz = sz + Fz;
    range = find(abs(sz-Tz)<tol);
    
    if isempty(range);
        error = 1;
    else
    splash = [sx(1,range(end)), sy(1,range(end))];
    error = norm(splash - [Tx, Ty]);
    end

    if error < 0.03;
        velocity = v0;
        break
    end
    
ax = gca;
ax.Clipping = 'off';
end    
ballTrajectory = [sx;sy;sz];
    if plot == 1
        plot3(sx,sy,sz, 'o');
    end

hold off
end

