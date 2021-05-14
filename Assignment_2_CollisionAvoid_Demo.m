close all
clear
clc

% plot trajectory
steps = 30;

UR3_ = UR3;
view(3);

%Drawback position = T0
% [###, 40, 120, 160, 90,0], [###, 0.69813, 2.09439, 2.7923, 1.57079, 0]
% [ 0,  0,  1, -0.1104;
%   1,  0,  0, 0.3495;
%   0,  1,  0, 0.0547;
%   0,  0,  0, 1      ]
%
%
%Throw Posiotion = T1
% [###, 65, 15, 55, 90, 0],   [###, 1.0471, 0.1745, 0.54105, 1.57079, 0]
% [0,  0,  1,  -0.1104;
%  1,  0.4225,  0.9064,  0.5057;
%  0,  0.9064,  -0.4225,  0.2351;
%  0,  0,  0,  1]
%Double Travel Position
%
% %Create goal transform for both arms to achieve
% %Zero arm link values as defaut
qUR3 = zeros(1,6);
% 
centerpnt = [0.2,0.3,0.125];
side = 0.25;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
plotOptions.plotFaces = false;
plotOptions.plotEdges= false;
side = 0.35;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

% %Find Link values that achieve goal transform
% newQ_UR3 = ikcon(UR3_.model, pose_UR3, q_UR3);
newQUR3 = [0, 0.34906585, 2, 0, 1.57079, 0];
newQUR3 = [0, 0.34906585, 2, (newQUR3(1,2)+ newQUR3(1,3)), 1.57079, 0];
%Model movement between both ultimate q-values into a large matrix
aniStepsUR3 = jtraj(qUR3,newQUR3,steps);
 
%File through q-values to 'animate' the arm to movement
for i = 1:1:size(aniStepsUR3,1)
       UR3_.model.animate(aniStepsUR3(i,:));
       pause(0.1);
end
ax = gca;
ax.Clipping = 'off';
%Place hold current posture to reference later



%% Trajectory Finding

%Display Ball
ballBag = RobotBall();
ballBag.ball{1}.base = UR3_.model.fkine(newQUR3);
animate(ballBag.ball{1},0);


%Find Target and plot ball curve
display(['Initial Trajectory Estimation']);


target = [0.5,0.4,0.3];
angleTraj = 35.5;

PlotTrajectory(UR3_, newQUR3,target, angleTraj,0);
pause(1);
oldQUR3 = newQUR3;

%Adjust position and calculate again
CollisionFlag = false;
display(['Aligning UR3']);
[newQUR3, xyUR3Angle, q1] = AdjustDirection(UR3_, oldQUR3,target,ballBag, false);
CollisionFlag  = IsCollision(UR3_.model,q1,faces,vertex,faceNormals);

if CollisionFlag == true
     count = 0;
     display([' ']);
     display([' ']);
     display(['There seems to be another player on the court.']);
     display(['They''re putting up a strong defence and blocking your path.']);
     while(1)
     prompt = 'Do you wish to proceed anyway? Y/N: ';
     type = input(prompt,'s');
        if type=='N' | type=='n'
            error('A collision was detected and the script was aborted.');
        elseif type=='Y' | type=='y'
            display(['Proceeding with collison avoidance.']);
            break;
        else
            display(['Input was not valid. Try again.']); 
            count = count + 1;
            if count > 2
                error('Too many incorrect inputs. Aborting sequence.');
            end
        end
    end
        
end

while CollisionFlag == true
newQUR3(1,2) = newQUR3(1,2) - 0.05
[newQUR3, xyUR3Angle, q1] = AdjustDirection(UR3_, newQUR3,target,ballBag, false);
CollisionFlag  = IsCollision(UR3_.model,q1,faces,vertex,faceNormals);
if CollisionFlag == false
    aniStepsUR3 = jtraj(oldQUR3,newQUR3,20);
     
     for i = 1:1:size(aniStepsUR3,1)
                UR3_.model.animate(aniStepsUR3(i,:));
                ballBag.ball{1}.base = UR3_.model.fkine(aniStepsUR3(i,:));
                animate(ballBag.ball{1},0);
                pause(0.1);
     end 
end
end

[newQUR3, xyUR3Angle, q1] = AdjustDirection(UR3_, newQUR3,target,ballBag);

%% check for setting back down

[ballTrajectory,velocity] = PlotTrajectory(UR3_, newQUR3, target,angleTraj,0);
pause(1);


    steps = 30;
    vSteps = 6;
    oldQUR3 = newQUR3;
    newQUR3 = [newQUR3(1,1), 0.69813, 2.09439, 2.7923, 1.57079, 0];
    aniStepsUR3 = jtraj(oldQUR3,newQUR3,steps);
     
     for i = 1:1:size(aniStepsUR3,1)
                UR3_.model.animate(aniStepsUR3(i,:));
                ballBag.ball{1}.base = UR3_.model.fkine(aniStepsUR3(i,:));
                animate(ballBag.ball{1},0);
                pause(0.1);
     end 

 oldQUR3 = newQUR3;
 thisPoseUR3 = UR3_.model.fkine(oldQUR3);
 qT = [newQUR3(1,1), 0.52359, 0.9250, 0, 1.57079, 0];
 qT = [newQUR3(1,1), 0.52359, 0.9250, qT(1,2)+ qT(1,3) - deg2rad(angleTraj), 1.57079, 0];
 nextPoseUR3 = UR3_.model.fkine(qT);
 [ballTrajectory,velocity] = PlotTrajectory(UR3_, qT,target,angleTraj,1);

 [dis,vel,acc] = lspb(0,3,vSteps,velocity);
 aniStepsUR3 = ctraj(thisPoseUR3, nextPoseUR3, dis);    
 oldQUR3 = newQUR3;

         for i = 1:1:size(aniStepsUR3,3)
            newQUR3 = ikcon(UR3_.model, aniStepsUR3(:,:,i), newQUR3);
            UR3_.model.animate(newQUR3);  
            ballBag.ball{1}.base = UR3_.model.fkine(newQUR3);
            animate(ballBag.ball{1},0);
            pause(0.1);
            display(['End effector Velocity is ' ,num2str(vel(i)),' m/s.']);
             if vel(i) >= velocity 
                 break;
             end
         end
         
         % Ball being thrown animation
        display(['Secondary Trajectory Estimation']);
        ThrowBall(ballBag, ballTrajectory);
        [bounce] = BallBounce(ballBag, 3);
        ThrowBall(ballBag, bounce);
       
        
        
        
        