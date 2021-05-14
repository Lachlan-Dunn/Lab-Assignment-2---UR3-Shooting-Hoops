function [q, z] = Search(robot, hoop,qR, backB, cam)
%Find Target and plot ball curve
% this was used to test base
% targetBase = Hoop_.model.base;
% targetBase(3, 4) = 0.5;
% targetBase(2, 4) = -0.1;
% target1 = targetBase(1:3,4)';
% target2 = 0.1*target1;
% target = [target1; target2]

% backB = hoop.model.fkine(0);
% backB(3, 4) = backB(3, 4)+0.2417;
% backB = backB(1:3, 4);

z = backB(3,:);
pStar = [512, 512];
%cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, 'resolution', [1024, 1024], ...
%    'centre', [512, 512], 'name', 'effincam');
fps = 25;
lambda = 0.8;
depth = z;
camT0 = robot.model.fkine(qR);
cam.T = camT0;
cam.plot_camera('Tcam', camT0, 'label', 'scale', 0.1)
p = cam.plot(backB, 'Tcam', camT0);
lighting gouraud
light

cam.clf
cam.plot(pStar', '*');
cam.hold(true);
cam.plot(backB, 'Tcam', camT0, 'o');
pause(0.5);
cam.hold(true);
cam.plot(backB);

%Initialise display arrays
vel_p = [];
uv_p = [];
history = [];
ksteps = 0;

while true
    ksteps = ksteps + 1;
    uv = cam.plot(backB)
    e = pStar - uv';
    e = e(:);
    Zest = [];
    
    if isempty(depth)
        pt = homtrans(inv(Tcam), backB);
        J = cam.visjac_p(uv, pt(3,:));
        
    elseif ~isempty(Zest)
        J = cam.visjac_p(uv, Zest);
    
    else
        J = cam.visjac_p(uv, depth);
        
    end

    try 
        v = lambda*pinv(J)*e;
       
    catch
        status = -1;
        return
    end
    
    fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
    
    rJ = robot.model.jacobn(qR);
    rJinv = pinv(rJ);
    qp = rJinv*v;
    
    %Maximum angular velocity cannot exceed 180 degrees/s
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi;
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end
    %Update UR3 Model     
    q = qR + (1/fps).*qp;
    
    %add collision
    
    robot.model.animate(q');
    %Update the ball position
    %ballBag.ball{1}.base = robot.model.fkine(qR);
    %animate(ballBag.ball{1},0);
    
    camTc = robot.model.fkine(q);
    cam.T = camTc;   
    drawnow        
    pause(1/fps)

    if ~isempty(200) && (ksteps > 200)
        break;
    end
    
        %update current joint position
        qR = q;
       
 end %loop finishes

end