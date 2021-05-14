classdef RobotBall < handle
    %ROBOTROBOT A way of creating a herd of robot brick
    %   The cows can be moved around randomly. It is then possible to query
    %   the current location (base) of the cows.
    
    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 15;
    end
    
    properties
        %> Number of balls
        ballCount = 1;
        
        %Offset of the bricks z value
        %And flip so the origin is facing up
        offset = 0.033;
        flip = pi;
        
        
        %> A cell structure of \c bricCount brick models
        ball;
        
        %> workSite in meters
        workSite = [2,2];        
        
        %> Dimensions of the workspace in regard to the site size
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function this = RobotBall(ball_pose)
            if 1 < nargin
                this.ball_pose = ball_pose;
            end
            
            this.workspaceDimensions = [-this.workSite(1)/2, this.workSite(1)/2 ...
                                       ,-this.workSite(2)/2, this.workSite(2)/2 ...
                                       ,0,this.maxHeight];
            offset = 0.033;
            flip = pi;

            % Create the required number of balls
            for i = 1:this.ballCount
                display(['Placing Ball in UR3s tool']);
                this.ball{i} = this.GetBall(['ball',num2str(i)]);
                
                switch i
                    case 1
                        this.ball{i}.base = transl(0.1, 0.1, 0 + offset);
                    case 2
                        this.ball{i}.base = transl(0.1, 0.1, 0 + offset)
                    case 3
                        this.ball{i}.base = transl(0.1, 0.1, 0 + offset)
                    case 4
                        this.ball{i}.base = transl(0.1, 0.1, 0 + offset)
                    case 5
                        this.ball{i}.base = transl(0.1, 0.1, 0 + offset)
                    case 6
                        this.ball{i}.base = transl(0.1, 0.1, 0 + offset)
                    case 7
                        this.ball{i}.base = transl(0.1, 0.1, 0 + offset)
                    case 8
                        this.ball{i}.base = transl(0.1, 0.1, 0 + offset)
                    case 9
                        this.ball{i}.base = transl(0.1, 0.1, 0 + offset)
                    otherwise
                        this.ball{i}.base = transl(0.1, 0.1, 0 + offset)
                        
                end
                
                % Plot 3D model
                plot3d(this.ball{i},0,'workspace',this.workspaceDimensions,'view',[-20,20],'delay',0);
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end

           axis equal;
        end
        
        function delete(this)
%             cla;
        end
        
         %% Throw Ball
    function ThrowBall(ballBag, traj)
        for i = 1:2:size(traj,2)
            aniStepsBall = transl(traj(1,i), traj(2,i), traj(3,i));
            ballBag.ball{1}.base = aniStepsBall;
            animate(ballBag.ball{1},0);
            pause(0.001);
        end
    end
        %% Ball Bounce
        function [ballBounce] = BallBounce(point, t)
            x = point.ball{1}.base(1,4);
            y = point.ball{1}.base(2,4);
            z = point.ball{1}.base(3,4);
            a = linspace(x,x);
            b = linspace(y,y);
            c = linspace(z,0);
            ballBounce = [a;b;c];
            v0 = 4.42703*z^0.5;
          for i = 1:1:t
             v0 = v0/1.3;
            g = 9.8;
            tges=(2*v0.*sin(88.*(pi/180)))/g;
            t =linspace(0,tges);
            v_xy = v0.*cos(90.*(pi/180));
            v_z = v0.*sin(90.*(pi/180));
            sx = a;
            sy = b;
            sz = v_z.*t-0.5*g.*t.^2;
            ph = [sx;sy;sz];
            ballBounce = [ballBounce ph];
            
          end
    end
        %% Moving Ball
        function MoveBall(this, ballIndex)

                animate(this.ball{ballIndex},0);               
            % Do the drawing once for each interation for speed
            drawnow();
        end    
      
    end
    
    methods (Static)
        %% Get Ball
        function model = GetBall(name)
            if nargin < 1
                name = 'Ball';
            end
            [faceData,vertexData] = plyread('ball.ply','tri');
            L1 = Link('alpha',-pi/2,'a',0,'d',0.3,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2);% + 0.4;
            model.points = {vertexData,[]};
             
        end
   
        
    end    
end

