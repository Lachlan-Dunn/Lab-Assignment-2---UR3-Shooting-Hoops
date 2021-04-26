classdef UR3 < handle
    properties
    
        model;
        workspace = [-2 2 -2 2 0 2];   
        
        %Check if gripper was requested
        useGripper = false;        
    end   
    methods
function myUR3 = UR3(useGripper)
    if nargin < 1
        useGripper = false;
    end
    myUR3.useGripper = useGripper;
    
myUR3.Grab_UR3_Parameters();

myUR3.Build_UR3_Robot();
end

%% Grab UR3 Parameters
function Grab_UR3_Parameters(myUR3)

        pause(0.001);
        name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];

        L1 = Link('d', 0.1519, 'a', 0, 'alpha', -pi/2 ,'offset', 0, 'qlim', [-2*pi, 2*pi]);
        L2 = Link('d', 0.11985, 'a', 0.24365, 'alpha', 0, 'offset', -pi/2, 'qlim', [-pi/1.5, pi/2.5]);
        L3 = Link('d', -0.09285, 'a', 0.21325, 'alpha', pi, 'offset', 0, 'qlim', [-pi/1.5, pi/1.5]);
        L4 = Link('d', -0.0834, 'a', 0, 'alpha', -pi/2, 'offset', pi/2, 'qlim', [-pi, 240*pi/180]);
        L5 = Link('d', 0.0834, 'a', 0, 'alpha', -pi/2, 'qlim', [-25*pi/45, 25*pi/45]);;
        L6 = Link('d', 0.12, 'a', 0, 'alpha', 0, 'qlim', [-pi, pi]);
 
        myUR3.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
        myUR3.model.base = transl(0.28,0,0.5);
        %a = myUR3.model.qlim
end
%% Build UR3 Robot
function Build_UR3_Robot(myUR3)
    for linkIndex = 0:myUR3.model.n
        if myUR3.useGripper && linkIndex == myUR3.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR3Link',num2str(linkIndex),'Gripper.ply'],'tri'); 
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR3Link',num2str(linkIndex),'.ply'],'tri');
        end
        myUR3.model.faces{linkIndex+1} = faceData;
        myUR3.model.points{linkIndex+1} = vertexData;
    end

    myUR3.model.plot3d(zeros(1,myUR3.model.n),'noarrow','workspace',myUR3.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    myUR3.model.delay = 0;

    for linkIndex = 0:myUR3.model.n
        handles = findobj('Tag', myUR3.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end

%% Calculate Workspace
function Plot_UR3_Volume(myUR3)

    hold on
    stepRads = deg2rad(100);
    qlim_UR3 = myUR3.model.qlim;
    UR3_pCS = prod(floor((qlim_UR3(1:5,2)-qlim_UR3(1:5,1))/stepRads + 1));
    UR3_pC = zeros(UR3_pCS,3);
    counter = 1;
    tic
    display(['Calculating...  This will only be a second.']);
    for q1 = qlim_UR3(1,1):stepRads:qlim_UR3(1,2)
        for q2 = qlim_UR3(2,1):stepRads:qlim_UR3(2,2)
            for q3 = qlim_UR3(3,1):stepRads:qlim_UR3(3,2)
                for q4 = qlim_UR3(4,1):stepRads:qlim_UR3(4,2)
                    for q5 = qlim_UR3(5,1):stepRads:qlim_UR3(5,2)
                        q6 = 0;
                            q = [q1,q2,q3,q4,q5,q6];
                            tr = myUR3.model.fkine(q);   
                            if 0.5 < tr(3,4)
                              UR3_pC(counter,:) = tr(1:3,4)';
                            end
                            counter = counter + 1; 

                    end
                end
            end
        end
    end
    display(['Done']);
    %plot3(UR3_pC(:,1),UR3_pC(:,2),UR3_pC(:,3),'o');

    UR3_shell_x = UR3_pC(:,1);
    UR3_shell_y = UR3_pC(:,2);
    UR3_shell_z = UR3_pC(:,3);
    UR3_shell_zS = size(UR3_shell_z);

    for i = 1:1:UR3_shell_zS(1)
        if 0 >= UR3_shell_z(i,1)
            UR3_shell_z(i,1) = 0.5;
        end
    end

    [k,UR3_v] = convhull(UR3_shell_x,UR3_shell_y,UR3_shell_z);
    trisurf(k,UR3_shell_x,UR3_shell_y,UR3_shell_z,'FaceColor','cyan');
    alpha(0.5);
    display(['Volume of the UR3 Robot workspace using "convhull" is ',num2str(UR3_v),'m^3']);
    axis equal;

    UR3_max_x = max(UR3_pC(:,1));
    UR3_x = min(UR3_pC(:,1));
    UR3_av_x = (UR3_max_x - UR3_x)/2;
    UR3_est_v = (4*pi*UR3_av_x^3)/3;
    display(['Volume of the UR3 Robot workspace using "estimated sphere volume" is ',num2str(UR3_est_v),'m^3']);
    hold off
end
    end
end