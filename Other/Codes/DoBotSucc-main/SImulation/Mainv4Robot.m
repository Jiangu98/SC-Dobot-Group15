%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%VARIABLES%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef SafeCoExpressSorting < handle
    properties
        systemUR3; %% Enable UR3 Robot
        systemUR5; %% Enable UR5 Robot
        
        %% Initial Ur5q_i joint configuration to ensure no initial collision
        ur5q_i = [-0.8 1.5708 0 -1.5708 1.5708 0 0];
        %% Initial Ur3q_i joint configuration to ensure no initial collision
        DoBotq_i = [0 0 0 0];
        
        GripInitialisation = false; %% Dont enable Grip otherwise crashes
        scale = 0.45; %% Scale for workspace
        useGripper = false; %% Dont enable gripper
        space = [-5 5 -3 3 -2 2]; %% Workspace Dimensions
        DataPointsSizeUR3; %% Collect Datapoints for workpsace - Sizing
        DataPointsUR3 = {};%% Collect Datapoints for workpsace 
        DataPointsSizeUR5; %% Collect Datapoints for workpsace - Sizing
        DataPointsUR5 = {}; %% Collect Datapoints for workpsace
        
        
        
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%VARIABLES%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%DECLARATIONS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function start = SafeCoExpressSorting(initpos,spawnur5,Brick1,Brick2,Brick3,Brick4,Brick5,Brick6,Brick7,Brick8,Brick9)
            disp('Initiialising Properties...')
            if nargin == 0; %% returns the number of function input arguments given in the call to the currently executing function
                
                initx = 0.4; %% Initial x variable for UR5 position
                inity = -1.7; %% Initial y variable for UR5 position
                
                initpos = transl(-0.1,0,0); %% UR3 Spawn Location
                
                set(0,'DefaultFigureWindowStyle','docked') %% Pop up on MATLAB
                spawnur5 = transl(initx,inity,0); %% Inital UR5 Spawn Location
                spawn = transl(0,0,0); %% Initial Table Spawn Location
                
                
                %%%%UR5 Bricks%%%%
                Brick1 = [0.8,-1.6,0]
                Brick2 = [0.22,-1.2,0]
                Brick3 = [0.5,-1.2,0]
                Brick4 = [-0.2,-1.2,0]
                Brick5 = [-0.82,-1.6,0]
                %%%%UR5 Bricks%%%%
                
                %%%%UR3 Bricks%%%%
                Brick6 = [0.2,-0.2,0]
                Brick7 = [0,-0.2,0]
                Brick8 = [-0.4,-0.3,0]
                Brick9 = [-0.7,-0.2,0]
                %%%%UR3 Bricks%%%%
            end
            %% Grabbing X , Y , Z values from InitPos(Table) and placing on matrix
            xspace = initpos(1,4); %%Establish my workspace using initial x position
            yspace = initpos (2,4); %%Establish my workspace using initial y position
            zspace = initpos (3,4); %%Establish my workspace using initial z position
            
            %% Start Workspace
            start.space = [-5+xspace 4+xspace -3+yspace 3+yspace -0.55+zspace 2+zspace]; %%Workspace
            %% Start Setting Up Environment
            start.SetupEnvironment(initpos,spawn);
            %% Initialise and Start UR3
            start.InitialiseUR3(initpos);
            start.MakeUR3();
            %% Initialise and Start UR5
           % start.InitialiseUR5(spawnur5);
            %start.MakeUR5();
            %% Generate Bricks and their final Location
           % start.GenerateBricks(Brick1,Brick2,Brick3,Brick4,Brick5,Brick6,Brick7,Brick8,Brick9);
            %% Animate the robots to place bricks into position
            %disp('Press Enter to begin')
            %pause();
           % start.AnimateRobots(spawnur5,initx,inity,Brick1,Brick2,Brick3,Brick4,Brick5,Brick6,Brick7,Brick8,Brick9);
            %disp('Generate the workspace (Radius) by pressing Enter');
            %pause();
            %% Generate the Work Radius
           % start.GenerateWorkRad();
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%DECLARATIONS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%LOAD ENVIRONMENT%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function SetupEnvironment(start,initpos,spawn)
            disp('Forming Environment...')
            %% Setting up table
            [f,v,data] = plyread('table2.ply','tri');

            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            %% Uses UR3 X and Y coordinates and puts into matrix
            xstable = spawn(1,4);
            ystable = spawn(2,4);
            
            %% Uses Table and Sets X, Y and Z variables into Matrix
            XSpacing = initpos (1,4); 
            YSpacing = initpos (2,4); 
            ZSpacing = initpos (3,4);
            
            
            trisurf(f,v(:,1)+ xstable,v(:,2) + ystable-1, v(:,3)+ ZSpacing-0.3, 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on;
            camlight;
            axis equal;
            view(3);
            
           
            %% Setting up Wall stand
            [f,v,data] = plyread('wall.ply','tri'); 
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255; 
            for XSpacing = [0,0]
                for YSpacing = [0,0]
                    trisurf(f,v(:,1)+ XSpacing-2,v(:,2) + YSpacing-3, v(:,3)+ ZSpacing+0.5 ... 
                        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
                end
            end
            
            %% Setting up Floor
            surf([-4,-4;4,4],[-3,3.5;-3,3.5],[-0.5,-0.5;-0.5,-0.5],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            
            [f,v,data] = plyread('fance.ply','tri'); 
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255; 
            for XSpacing = [0,0]
                for YSpacing = [0,0]
                    trisurf(f,v(:,1)+ XSpacing-0.1,v(:,2) + YSpacing+0.8, v(:,3)+ ZSpacing-0.5 ... 
                        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
                end
            end
            
            %% Setting up Fire Estinguisher
            [f,v,data] = plyread('extinguisher.ply','tri'); 
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255; 
            for XSpacing = [0,0]
                for YSpacing = [0,0]
                    trisurf(f,v(:,1)+ XSpacing+2,v(:,2) + YSpacing+2, v(:,3)+ ZSpacing-0.45 ... 
                        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
                end
            end
            
            %% Setting up Fence stand
            [f,v,data] = plyread('fance.ply','tri'); %%Import Stand
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255; 
            for XSpacing = [0,0]
                for YSpacing = [0,0]
                    trisurf(f,v(:,1)+ XSpacing-0.1,v(:,2) + YSpacing- 2.7, v(:,3)+ ZSpacing-0.5 ... 
                        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
                end
            end
            
            %% Setting up Fence stand
            [f,v,data] = plyread('fance2.ply','tri'); %%Import Stand
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255; 
            for XSpacing = [0,0]
                for YSpacing = [0,0]
                    trisurf(f,v(:,1)+ XSpacing+1.2,v(:,2) + YSpacing-1, v(:,3)+ ZSpacing-0.5 ... 
                        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
                end
            end
            
            %% Setting up Fence stand
            [f,v,data] = plyread('fance2.ply','tri'); %%Import Stand
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255; 
            for XSpacing = [0,0]
                for YSpacing = [0,0]
                    trisurf(f,v(:,1)+ XSpacing-1.2,v(:,2) + YSpacing-1, v(:,3)+ ZSpacing-0.5 ... 
                        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
                end
            end
            
            disp('Environment Loaded...')
            disp('Initialise UR5...')
            
            
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%LOAD ENVIRONMENT%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function InitialiseUR3(start, initpos)
            name = 'DoBot'; %%Named 
            disp('Initialising DoBot')
            %% UR3 DH Paramaters found online
            L1 = Link('d', 0.09, 'a', 0, 'alpha', 0, 'qlim', [-3*pi/4, 3*pi/4]);
            L2 = Link('d', 0,'a', 0, 'alpha', 0, 'qlim', [0, 17*pi/36]);
            L3 = Link('d', 0.05346, 'a',-0.148, 'alpha', 0, 'qlim', [-pi/18, 19*pi/36]);
            L4 = Link('d', -0.09, 'a', -0.16395, 'alpha', 0, 'qlim', [-pi/2, pi/2]);
                      
            %% Spawn edits the base revolve below , revolves x on 1x4 matrix and x on 1x3 matrix making 3:4 matrix
            BaseZRevolve = trotz(pi/2);
            BaseStart = [BaseZRevolve(1:4, 1:3) initpos(1:4, 4)];
            %% Joins the links through serial Link, Linking L1-6 together
            start.DoBot = SerialLink([L1 L2 L3 L4],'name',name, 'base', BaseStart);
        end 
        function MakeUR3(start)
            for linkno = 0:start.DoBot.n
                [ faceData, vertexData, plyData{linkno+1} ] = plyread(['UR3Link',num2str(linkno),'.ply'],'tri'); %#ok<AGROW>
                start.systemUR3.faces{linkno+1} = faceData;
                start.systemUR3.points{linkno+1} = vertexData;
            end
            
            %% Code below displays robot, first enabling wrist, xyz, arrow
            start.DoBot.plotopt3d = {'wrist', 'xyz', 'arrow'};
            %% Plots Joint configuration at current state to workspace
            start.DoBot.plot3d(start.DoBotq_i, 'workspace',start.space, 'scale', start.scale) 
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            start.DoBot.delay = 0;
            
            %% Attempt to colour arm in the proper way if vertexes have colour points, available in ply file
            for linkno = 0:start.DoBot.n
                handles = findobj('Tag', start.DoBot.name);
                h = get(handles,'UserData');
                try
                    h.link(linkno+1).Children.FaceVertexCData = [plyData{linkno+1}.vertex.red ...
                        , plyData{linkno+1}.vertex.green ...
                        , plyData{linkno+1}.vertex.blue]/255;
                    h.link(linkno+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BRICKS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function InitialiseUR5(start,spawnur5)
            RobotUR5x = spawnur5(1,4);
            RobotUR5y = spawnur5(2,4);
            RobotUR5z = spawnur5(3,4);
            
            pause(0.001); %%Creates Unique Name for LinearUR_5_Linear
            name = ['LinearUR_5_',datestr(now,'yyyymmddTHHMMSSFFF')];
            
            
            % Create the UR5 systemUR5 mounted on a linear rail
            L(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            L(2) = Link([0      0.1599  0       -pi/2   0]);
            L(3) = Link([0      0.1357  0.425   -pi     0]);
            L(4) = Link([0      0.1197  0.39243 pi      0]);
            L(5) = Link([0      0.093   0       -pi/2   0]);
            L(6) = Link([0      0.093   0       -pi/2	0]);
            L(7) = Link([0      0       0       0       0]);
            
            %%Following Joint limits
            L(1).qlim = [-0.8 0];
            L(2).qlim = [-360 360]*pi/180;
            L(3).qlim = [-90 90]*pi/180;
            L(4).qlim = [-170 170]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;
            L(7).qlim = [-360 360]*pi/180;
            
            L(3).offset = -pi/2;
            L(5).offset = -pi/2;
            
            start.systemUR5 = SerialLink(L,'name',name);
            
            % Rotate robot to the correct orientation
            start.systemUR5.base = transl(RobotUR5x,RobotUR5y,RobotUR5z)*start.systemUR5.base * trotx(pi/2) * troty(pi/2);
        end
        function MakeUR5(start)
            for linkIndex = 0:start.systemUR5.n
                if start.useGripper && linkIndex == start.systemUR5.n
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR5Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
                else
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                end
                start.systemUR5.faces{linkIndex+1} = faceData;
                start.systemUR5.points{linkIndex+1} = vertexData;
            end
            
            % Display robot
            start.systemUR5.plot3d(zeros(1,start.systemUR5.n),'noarrow','workspace',start.space);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            start.systemUR5.delay = 0;
            
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:start.systemUR5.n
                handles = findobj('Tag', start.systemUR5.name);
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
        function GenerateBricks(start,Brick1,Brick2,Brick3,Brick4,Brick5,Brick6,Brick7,Brick8,Brick9)
            
            %%Place Bricks using co-ordinates fed in start function
            
            %%%%UR5 Bricks%%%%
            PlaceObject('Brick.ply',Brick1);
            PlaceObject('Brick.ply',Brick2);
            PlaceObject('Brick.ply',Brick3);
            PlaceObject('Brick.ply',Brick4);
            PlaceObject('Brick.ply',Brick5);
            %%%%UR5 Bricks%%%%
            
            
            %%%%UR3 Bricks%%%%
            PlaceObject('Brick.ply',Brick6);
            PlaceObject('Brick.ply',Brick7);
            PlaceObject('Brick.ply',Brick8);
            PlaceObject('Brick.ply',Brick9);
            %%%%UR3 Bricks%%%%            
        end
        function AnimateRobots(start,spawnur5,initx,inity,Brick1,Brick2,Brick3,Brick4,Brick5,Brick6,Brick7,Brick8,Brick9)
            
            %% Initial Ur5q_i joint configuration to ensure no initial collision
            ur5q_i = [-0.8 1.5708 0 -1.5708 1.5708 0 0];
            %% Initial Ur3q_i joint configuration to ensure no initial collision
            ur3q_i = [0 -pi/2 pi/2 -pi/2 -pi/2 0];
            
            steps = 50;
            
            %% Place Bricks in designated positions infront of UR5 Robot
            
            %%%%UR5 Bricks%%%%
            Wall1 = [initx,inity,0]+[0,1.1344,0]
            Wall2 = [initx,inity,0]+[-0.532,1.1344,0]
            Wall3 = [initx,inity,0]+[-0.266,1.1344,0.0667]
            Wall4 = [initx,inity,0]+[0,1.1344,0.1334]
            Wall5 = [initx,inity,0]+[-0.532,1.1344,0.1334]
            %%%%UR5 Bricks%%%%
            
            %%%%UR3 Bricks%%%%
            Wall6 = [initx,inity,0]+[-0.266,1.1344,0]
            Wall7 = [initx,inity,0]+[-0.532,1.1344,0.0667]
            Wall8 = [initx,inity,0]+[-0,1.1344,0.0667]
            Wall9 = [initx,inity,0]+[-0.266,1.1344,0.1334]
            %%%%UR3 Bricks%%%%
            
            %% Bricks 1 (UR5) and Brick 6 (UR3)
            %%%Finding joint angles from initial post
            Initial_Brick_1_pos = transl(Brick1) * trotx(pi) 
            Final_Brick_1_pos = spawnur5+transl(0.266,0.1344,0) * trotx(pi)
            Initial_Brick_6_pos = transl(Brick6) * trotx(pi) 
            Final_Brick_6_pos = spawnur5+transl(0.800,0.1344,0.0667) * trotx(pi)
            
            Brick_1_ikcon = start.systemUR5.ikcon(Initial_Brick_1_pos,ur5q_i) %% Calculate Ikron from UR5q_i to Initial Brick 1 pose
            %%%Trajectory from initial Joint angle to Brick 1
            Brick_1_Trajectory = jtraj(ur5q_i,Brick_1_ikcon,steps)
            Brick_6_ikcon = start.systemUR3.ikcon(Initial_Brick_6_pos,ur3q_i)
            Brick_6_Trajectory = jtraj(ur3q_i,Brick_6_ikcon,steps)
            
            disp('Picking up Bricks 1 and 6')
            for i = 1:steps
                animate(start.systemUR5,Brick_1_Trajectory(i,:))
                animate(start.systemUR3,Brick_6_Trajectory(i,:))
                drawnow();
                pause(0.01)
            end
            ur5_B1_w = start.systemUR5.ikcon(Final_Brick_1_pos,Brick_1_ikcon)
            Traj5_1_w = jtraj(Brick_1_ikcon,ur5_B1_w,steps)
            
            ur3_B6_w = start.systemUR3.ikcon(Final_Brick_6_pos,Brick_6_ikcon)
            Traj3_6_w = jtraj(Brick_6_ikcon,ur3_B6_w,steps)
           
            for i=1:steps
                animate(start.systemUR5,Traj5_1_w(i,:))
                animate(start.systemUR3,Traj3_6_w(i,:))
                drawnow();
                pause(0.01)
            end
            PlaceObject('Brick.ply',Wall1);
            PlaceObject('Brick.ply',Wall6);
            disp('Bricks 1 and 6 placed')
      
            
             %% Bricks 2 (UR5) and Brick 7 (UR3)
            Initial_Brick_2_pos = transl(Brick2) * trotx(pi)
            Final_Brick_2_pos = spawnur5 + transl(0.532,0.1344,0) * trotx(pi)
            Initial_Brick_7_pos = transl(Brick7) * trotx(pi) %%replace transl w/ Brick 1
            Final_Brick_7_pos = spawnur5+transl(0.266,0.1344,0.1334) * trotx(pi) %%replace transl w/ Brick 2
            
            Brick_2_ikcon = start.systemUR5.ikcon(Initial_Brick_2_pos,ur5_B1_w)
            Brick_2_Trajectory = jtraj(ur5_B1_w,Brick_2_ikcon,steps)
            Brick_7_ikcon = start.systemUR3.ikcon(Initial_Brick_7_pos,ur3_B6_w)
            Brick_7_Trajectory = jtraj(ur3_B6_w,Brick_7_ikcon,steps)
            disp('Picking up Bricks 2 and 7')
            for i = 1:steps
                animate(start.systemUR5,Brick_2_Trajectory(i,:))
                animate(start.systemUR3,Brick_7_Trajectory(i,:))
                drawnow();
                pause(0.01)
            end
            
            ur5_B2_w = start.systemUR5.ikcon(Final_Brick_2_pos,Brick_2_ikcon)
            Traj5_2_w = jtraj(Brick_2_ikcon,ur5_B2_w,steps)
            ur3_B7_w = start.systemUR3.ikcon(Final_Brick_7_pos,Brick_7_ikcon)
            Traj3_7_w = jtraj(Brick_7_ikcon,ur3_B7_w,steps)
           
            for i=1:steps
                animate(start.systemUR5,Traj5_2_w(i,:))
                animate(start.systemUR3,Traj3_7_w(i,:))
                drawnow();
                pause(0.01)
            end
            PlaceObject('Brick.ply',Wall2);
            PlaceObject('Brick.ply',Wall7);
            disp('Brick 2 and 7 placed')
            %%%%%%%%%%%%%%BRICK 2%%%%%%%%%%%%%%
            
            %% Bricks 3 (UR5) and Brick 8 (UR3)
            Initial_Brick_3_pos = transl(Brick3) * trotx(pi)
            Final_Brick_3_pos = spawnur5 + transl(0.800,0.1344,0) * trotx(pi)
            Initial_Brick_8_pos = transl(Brick8) * trotx(pi); %%replace transl w/ Brick 1
            Final_Brick_8_pos = spawnur5+transl(0.532,0.1344,0.1334) * trotx(pi) %%replace transl w/ Brick 2
            
            Brick_3_ikcon = start.systemUR5.ikcon(Initial_Brick_3_pos,ur5_B2_w)
            Brick_3_Trajectory = jtraj(ur5_B2_w,Brick_3_ikcon,steps)
            Brick_8_ikcon = start.systemUR3.ikcon(Initial_Brick_8_pos,ur3_B7_w)
            Brick_8_Trajectory = jtraj(ur3_B7_w,Brick_8_ikcon,steps)
            disp('Picking up Bricks 3 and 8')
            
            for i = 1:steps
                animate(start.systemUR5,Brick_3_Trajectory(i,:))
                animate(start.systemUR3,Brick_8_Trajectory(i,:))
                drawnow();
                pause(0.01)
            end
            
            ur5_B3_w = start.systemUR5.ikcon(Final_Brick_3_pos,Brick_3_ikcon)
            Traj5_3_w = jtraj(ur5_B3_w,ur5_B3_w,steps)
            ur3_B8_w = start.systemUR3.ikcon(Final_Brick_8_pos,Brick_8_ikcon)
            Traj3_8_w = jtraj(ur3_B8_w,ur3_B8_w,steps)
            
            for i=1:steps
                animate(start.systemUR5,Traj5_3_w(i,:))
                animate(start.systemUR3,Traj3_8_w(i,:))
                drawnow();
                pause(0.01)
            end
            PlaceObject('Brick.ply',Wall3)
            PlaceObject('Brick.ply',Wall8)
            disp('Brick placed')

            
            %% Bricks 4 (UR5) and Brick 9 (UR3)
            Initial_Brick_4_pos = transl(Brick4) * trotx(pi) %%replace transl w/ Brick 1
            Final_Brick_4_pos = spawnur5+transl(0.266,0.1344,0.0667) * trotx(pi) %%replace transl w/ Brick 2
            Initial_Brick_9_pos = transl(Brick9) * trotx(pi) %%replace transl w/ Brick 1
            Final_Brick_9_pos = spawnur5+transl(0.800,0.1344,0.1334) * trotx(pi) %%replace transl w/ Brick 2
            
            disp('Picking up Bricks 4 and 9')
            
            Brick_4_ikcon = start.systemUR5.ikcon(Initial_Brick_4_pos,ur5_B3_w)
            Brick_4_Trajectory = jtraj(ur5_B3_w,Brick_4_ikcon,steps)
            Brick_9_ikcon = start.systemUR3.ikcon(Initial_Brick_9_pos,ur3_B8_w)
            Brick_9_Trajectory = jtraj(ur3_B8_w,Brick_9_ikcon,steps)
            
            for i = 1:steps
                animate(start.systemUR5,Brick_4_Trajectory(i,:))
                animate(start.systemUR3,Brick_9_Trajectory(i,:))
                drawnow();
                pause(0.01)
            end
            ur5_B4_w = start.systemUR5.ikcon(Final_Brick_4_pos,Brick_4_ikcon)
            Traj5_4_w = jtraj(Brick_4_ikcon,ur5_B4_w,steps)
            ur3_B9_w = start.systemUR3.ikcon(Final_Brick_9_pos,Brick_9_ikcon)
            Traj3_9_w = jtraj(Brick_9_ikcon,ur3_B9_w,steps)
            
            for i=1:steps
                animate(start.systemUR5,Traj5_4_w(i,:))
                animate(start.systemUR3,Traj3_9_w(i,:))
                drawnow();
                pause(0.01)
            end
            PlaceObject('Brick.ply',Wall4);
            PlaceObject('Brick.ply',Wall9);
            
            disp('Bricks 4 and 9 placed')
            
            
            %% Bricks 5 (UR5) 
            Initial_Brick_5_pos = transl(Brick5) * trotx(pi) %%replace transl w/ Brick 1
            Final_Brick_5_pos = spawnur5+transl(0.532,0.1344,0.0667) * trotx(pi) %%replace transl w/ Brick 2
            
            Brick_5_ikcon = start.systemUR5.ikcon(Initial_Brick_5_pos,ur5_B4_w)
            Brick_5_Trajectory = jtraj(ur5_B4_w,Brick_5_ikcon,steps)
            disp('Picking up Brick 5')
            for i = 1:steps
                animate(start.systemUR5,Brick_5_Trajectory(i,:))
                drawnow();
                pause(0.01)
            end
            ur5_B5_w = start.systemUR5.ikcon(Final_Brick_5_pos,Brick_5_ikcon)
            Traj5_5_w = jtraj(Brick_5_ikcon,ur5_B5_w,steps)
                    
            for i=1:steps
                animate(start.systemUR5,Traj5_5_w(i,:))
                
                drawnow();
                pause(0.01)
            end
            PlaceObject('Brick.ply',Wall5);
            disp('Brick placed')
           
        end        
        function GenerateWorkRad(start)
            s_UR3 = 2; %%Establish 2 steps of UR3
            
            %% Trying to find limits between joints (sizing)
            JLimUR3 = start.systemUR3.qlim; %% JLimUR3 stores the joint limits taking in qlim
            range = rad2deg(JLimUR3(1:2,2))-rad2deg(JLimUR3(1:2,1)); %% Range from limit of joint 2-1
            a = (range/s_UR3); %%Range divided by 2 steps (2 joints)
            start.DataPointsSizeUR3 = prod(((rad2deg(JLimUR3(1:2,2))-rad2deg(JLimUR3(1:2,1)))/s_UR3)); %%Product of elements, Limit of Joint 2 -1
            start.DataPointsUR3 = zeros(start.DataPointsSizeUR3, 3); %% Populate point cloud with zeros so its a matrix
            checker = 1; %%Works as a counter
            tic %% Timer starts 
            ur3_j3 = 0; %%Joint 3 is 1
            ur3_j4 = deg2rad(-90); %% UR3 Parameters
            ur3_j5 = deg2rad(90); %% UR3 Parameters
            ur3_j6 = 0; %%Joint 6 is 0 (natural)
            
         % UR3 locations on x, y and z axis obtained
            for ur3_j1 = 1:a %% Figuring limits between joints 2-1 as only jo
                for ur3_j2 = 1:a  
                   q = [ur3_j1,ur3_j2,ur3_j3,ur3_j4,ur3_j5,ur3_j6];
                    %Values being constant are joints 6 and 3, which calculate workspace radius                    q = [ur3_j1,ur3_j2,ur3_j3,ur3_j4,ur3_j5,ur3_j6];
                    tr = start.systemUR3.fkine(q); %%This refers to the EE on the x, y and Z axis, transforms using fkine
                   start.DataPointsUR3(checker,:) = tr(1:3,4)'; %% equals transform, Will check each joint
                    checker = checker + 1; %% Moves up the joints, iterates and checks against each column, transposing fkine
                    if mod(checker/start.DataPointsSizeUR3 * 100,1) == 0
                        display(['UR3: After ',num2str(toc),' seconds, completed ',num2str(checker/start.DataPointsSizeUR3 * 100),'% of poses']);
                    end
                end
            end
            
            %%Points under table is deleted, allowing unobtainable points deselected. Creates semi-circle.
            for i = 1:start.DataPointsSizeUR3 %%Anything below zero is deleted as unreachable/colision, being part of identity matrix
                if start.DataPointsUR3(i,3) < 0
                    start.DataPointsUR3(i,3)=0;
                end
            end
            
            ur3_workradius = plot3(start.DataPointsUR3(:,1), start.DataPointsUR3(:,2), start.DataPointsUR3(:,3), 'r');
            
            % Create a variable to store the possible xyz values that the robot arms can be located in
            
            stepRads= deg2rad(5);
            start.DataPointsSizeUR5 = prod(floor((start.systemUR5.qlim(2:3,2)-start.systemUR5.qlim(2:3,1))/stepRads + 1)) * floor(start.systemUR5.qlim(1,2)-start.systemUR5.qlim(1,1)/stepRads + 1);
            start.DataPointsUR5 = zeros(start.DataPointsSizeUR5, 3);
            checker = 1; % reset the checker value
            tic
            ur5_j4 = 0;
            ur5_j5 = deg2rad(90);
            ur5_j6 = deg2rad(-90);
            ur5_j7 = 0;
            % Obtain the xyz locations for UR5
            for ur5_j1 = start.systemUR5.qlim(1,1):stepRads:start.systemUR5.qlim(1,2)
                for ur5_j2 = start.systemUR5.qlim(2,1):stepRads:start.systemUR5.qlim(2,2)
                    for ur5_j3 = start.systemUR5.qlim(3,1):stepRads:start.systemUR5.qlim(3,2)
                        % Joint 4 to joint 7 are constant values to calculate workspace radius
                        q = [ur5_j1,ur5_j2,ur5_j3,ur5_j4,ur5_j5,ur5_j6, ur5_j7];
                        tr = start.systemUR5.fkine(q); %is the EE xyz location
                        start.DataPointsUR5(checker,:) = tr(1:3,4)';
                        checker = checker + 1;
                        if mod(checker/start.DataPointsSizeUR5 * 100,1) == 0
                            disp(['UR5: After ',num2str(toc),' seconds, completed ',num2str(checker/start.DataPointsSizeUR5 * 100),'% of poses']);
                        end
                    end
                end
                
            end
            %% Plotting radius - plots the data of first, second and third row of UR5 matrix, only due to point cloud above = 3 rows, x,y and z 
            ur5_workradius = scatter3(start.DataPointsUR5(:,1), start.DataPointsUR5(:,2), start.DataPointsUR5(:,3), 'bl');
            
            max_reachUR3 = max(start.DataPointsUR3(:,3)) %% UR3 Reach
            max_reachUR5 = max(start.DataPointsUR5(:,3)) %% UR5 Reach
            
            %% Volume converted to m^2 and divided by 2 as semi circle
            ur3Volume = (start.DataPointsSizeUR3*0.00001)/2; 
            ur5Volume = (start.DataPointsSizeUR5*0.00001)/2;
            
            disp(['UR3 VOLUME ', num2str(ur3Volume), 'm^3']);
            disp(['UR5 VOLUME ', num2str(ur5Volume), 'm^3']);
            
            disp('Delete Data Points by selecting Enter!');
            pause();
            
            delete(ur5_workradius); delete(ur3_workradius);
            clc
        end
        
    end
end



