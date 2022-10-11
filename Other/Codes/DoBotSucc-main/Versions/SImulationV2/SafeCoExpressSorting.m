%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%VARIABLES%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef SafeCoExpressSorting < handle
    properties
        running = true
        clear
        clc
        
            %%%%%%%%% brick setup matracies%%%%%%%%%
             rbricks = []; 
            rb_vert = []; % Red Brick Verticies
            rb_trans = []; % Red Brick Transforms
            gbricks = [];
            gb_vert = []; % Green Brick Verticies
            gb_trans = []; % Green Brick Transforms
            bbricks = [];
            bb_vert = []; % Blue Brick Verticies
            bb_trans = []; % Blue Brick Transforms
        
            RedtransVert = 0;
            GreentransVert = 0;
            BluetransVert = 0;
        %%%%%%%%%%end brick setup matr%%%%%
        DoBotSpawn
        %% Initial DoBot joint configuration to ensure no initial collision
        DoBotq_i = [0 0 0 0];

        GripInitialisation = false; %% Dont enable Grip otherwise crashes
        scale = 0.45; %% Scale for workspace
        useGripper = false; %% Dont enable gripper
        space = [-5 5 -3 3 -2 2]; %% Workspace Dimensions

    end

    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%VARIABLES%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%DECLARATIONS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function start = SafeCoExpressSorting(initpos)
            disp('Initiialising Properties...')
            if nargin == 0; %% returns the number of function input arguments given in the call to the currently executing function
                abc = 2
                initx = 0.4; %% Initial x variable for UR5 position
                inity = -1.7; %% Initial y variable for UR5 position

                initpos = transl(-0.6,-1,0); %% UR3 Spawn Location

                set(0,'DefaultFigureWindowStyle','docked') %% Pop up on MATLAB
                start.DoBotSpawn = transl(initx,inity,0); %% Inital UR5 Spawn Location
                spawn = transl(0,0,0); %% Initial Table Spawn Location

            end
            %% Grabbing X , Y , Z values from InitPos(Table) and placing on matrix
            xspace = initpos(1,4); %%Establish my workspace using initial x position
            yspace = initpos (2,4); %%Establish my workspace using initial y position
            zspace = initpos (3,4); %%Establish my workspace using initial z position

            %% Start Workspace
            start.space = [-5+xspace 4+xspace -3+yspace 3+yspace -0.55+zspace 2+zspace]; %%Workspace
            %% Start Setting Up Environment
            start.SetupEnvironment(initpos,spawn);
            %% Initialise and Start DoBot
           %start.InitialiseDoBot(initpos);
           %start.MakeDoBot();
            %start.Camera();
            %% Generate Bricks and their final Location
           % disp("press a key to begin")
            %pause();
            start.GenerateBricks();
            %start.Collision_detection(); % must change to dobot2
            start.Main()
%            pause(2)
%            disp("click a button to continue")
            %% Animate the robots to place bricks into position
            %             disp('Press Enter to begin')
            %             pause();
            %start.Collisions()
            %start.AnimateRobots(DoBotSpawn,initx,inity);
            
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

            %% Setting up Lego Warden
            [f,v,data] = plyread('LegoMan.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            for XSpacing = [0,0]
                for YSpacing = [0,0]
                    trisurf(f,v(:,1)+ XSpacing+2,v(:,2) + YSpacing+3, v(:,3)+ ZSpacing-0.45 ...
                        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
                end
            end

            %% Setting up Fire Estinguisher
%             [f,v,data] = plyread('extinguisher.ply','tri');
%             vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%             for XSpacing = [0,0]
%                 for YSpacing = [0,0]
%                     trisurf(f,v(:,1)+ XSpacing+2,v(:,2) + YSpacing+2, v(:,3)+ ZSpacing-0.45 ...
%                         ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
%                 end
%             end
% 
%             [f,v,data] = plyread('extinguisher.ply','tri');
%             vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%             for XSpacing = [0,0]
%                 for YSpacing = [0,0]
%                     trisurf(f,v(:,1)+ XSpacing+2,v(:,2) + YSpacing+2, v(:,3)+ ZSpacing-0.45 ...
%                         ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
%                 end
%             end
           
            [f,v,data] = plyread('Belt.ply','tri'); %%Import Stand
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            for XSpacing = [0,0]
                for YSpacing = [0,0]
                    trisurf(f,v(:,1)+ XSpacing-0.25,v(:,2) + YSpacing -1 , v(:,3)+ ZSpacing-0.25 ...
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
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%LOAD ENVIRONMENT%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         function InitialiseDoBot(start, initpos)
            name = 'DoBot'; %%Named
            disp('Initialising DoBot')
            %% UR3 DH Paramaters found online
            L1 = Link('d', 0.09, 'a', 0, 'alpha', pi/2, 'qlim', [-3*pi/4, 3*pi/4]);
            L2 = Link('offset',deg2rad(-32.5),'d', 0,'a', -0.152, 'alpha', 0, 'qlim', [0, 17*pi/36]);
            L3 = Link('offset',deg2rad(62.25),'d', 0, 'a',-0.147, 'alpha', 0, 'qlim', [-pi/18, 19*pi/36]);
            L4 = Link('offset',deg2rad(-30),'d', 0, 'a', -0.04, 'alpha', pi/2, 'qlim', [-pi/2, pi/2]);
            L5 = Link('offset',deg2rad(-30),'d', 0.06, 'a', 0, 'alpha', 0, 'qlim', [-pi/2, pi/2]);
            
            %% Spawn edits the base revolve below , revolves x on 1x4 matrix and x on 1x3 matrix making 3:4 matrix
            BaseZRevolve = trotz(pi/2);
            BaseStart = [BaseZRevolve(1:4, 1:3) initpos(1:4, 4)];
            %% Joins the links through serial Link, Linking L1-6 together
            start.DoBot = SerialLink([L1 L2 L3 L4 L5],'name',name, 'base',BaseStart);
        end
        function MakeDoBot(start)
            for linkno = 0:start.DoBot.n
                [ faceData, vertexData, plyData{linkno+1} ] = plyread(['DOBOT_Magician-Link',num2str(linkno),'.ply'],'tri'); %#ok<AGROW>
                start.DoBot.faces{linkno+1} = faceData;
                start.DoBot.points{linkno+1} = vertexData;
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
        function GenerateBricks(start)
            j = 0;
            i = 1;
            hold on
           counter = 0
        for counter = 0:1:3
            i = 1;
            j=0;
            while i <= 2 % Blocks
                start.rbricks(i) = PlaceObject('Rbrick.ply');
                start.gbricks(i) = PlaceObject('Gbrick.ply');
                start.bbricks(i) = PlaceObject('Bbrick.ply');
                i = i+1;
            end
            i = 1;
            while i <= 2 %Array of vertacies
                start.rb_vert(:,:,i) = get(start.rbricks(i), 'Vertices');
                disp(start.rb_vert)
                start.gb_vert(:,:,i) = get(start.gbricks(i), 'Vertices');
                start.bb_vert(:,:,i) = get(start.bbricks(i), 'Vertices');
                i = i+1;

            end
            i=1;
            while i <= 2 %array of Brick Transforms
                start.rb_trans(:,:,i) = trotz(rand()*2*pi)*transl(rand()*0.65,rand()*0.65,0);
                start.gb_trans(:,:,i) = trotz(rand()*2*pi)*transl(rand()*0.65,rand()*0.65,0);
                start.bb_trans(:,:,i) = trotz(rand()*2*pi)*transl(rand()*0.65,rand()*0.65,0);
                
                i = i+1;
            end

            start.rb_trans(:,:,1) = transl(0,-2,0);
            start.gb_trans(:,:,1) = transl(0,-2,0);
            start.bb_trans(:,:,1) = transl(0,-2,0);
            i = 1;

%             while i < 500 %loop 1000 times for test
% 
%                 j=1;
%                 while j <=2 % test moving all bricks at the same time
%                     start.rb_trans(:,:,j) = transl(-0.65+j*0.25,-1+(i*0.003),0)*trotz(0.02*i);
%                     start.gb_trans(:,:,j) = transl(-0.65+j*0.25,-1.5+(i*0.003),0)*trotz(0.02*i);
%                     start.bb_trans(:,:,j) = transl(-0.65+j*0.25,-2+(i*0.003),0)*trotz(0.02*i);
%                     j=j+1;
%                 end
%                 j=1;
%                 while j <=2 % each loop of main, put each brick where its transform array says it should be (animation purposes)
%                     start.RedtransVert = [start.rb_vert(:,:,j),ones(size(start.rb_vert(:,:,j),1),1)] * start.rb_trans(:,:,j)';
%                     start.GreentransVert = [start.gb_vert(:,:,j),ones(size(start.gb_vert(:,:,j),1),1)] * start.gb_trans(:,:,j)';
%                     start.BluetransVert = [start.bb_vert(:,:,j),ones(size(start.bb_vert(:,:,j),1),1)] * start.bb_trans(:,:,j)';
% 
%                     set(start.rbricks(j),'Vertices',start.RedtransVert(:,1:3));
%                     set(start.gbricks(j),'Vertices',start.GreentransVert(:,1:3));
%                     set(start.bbricks(j),'Vertices',start.BluetransVert(:,1:3));
% 
% 
%                     j=j+1;
%                 end
%                 drawnow()
%                 i;
%                 i = i + 1;
%             end

            
        end
        end
        function AnimateRobots(start,DoBotSpawn,initx,inity)

            %% Initial Ur5q_i joint configuration to ensure no initial collision

            %% Initial DoBot joint configuration to ensure no initial collision
            DoBotq_i = [0 0 0 0];
            %% Initial Ur3q_i joint configuration to ensure no initial collision


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
            Final_Brick_1_pos = DoBotSpawn+transl(0.266,0.1344,0) * trotx(pi)
            Initial_Brick_6_pos = transl(Brick6) * trotx(pi)
            Final_Brick_6_pos = DoBotSpawn+transl(0.800,0.1344,0.0667) * trotx(pi)

            Brick_1_ikcon = start.DoBot.ikcon(Initial_Brick_1_pos,DoBotq_i) %% Calculate Ikron from UR5q_i to Initial Brick 1 pose
            %%%Trajectory from initial Joint angle to Brick 1
            Brick_1_Trajectory = jtraj(DoBotq_i,Brick_1_ikcon,steps)
            %Brick_6_ikcon = start.systemUR3.ikcon(Initial_Brick_6_pos,ur3q_i)
            %Brick_6_Trajectory = jtraj(ur3q_i,Brick_6_ikcon,steps)

            disp('Picking up Bricks 1 and 6')
            for i = 1:steps
                animate(start.DoBot,Brick_1_Trajectory(i,:))
                %animate(start.systemUR3,Brick_6_Trajectory(i,:))
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
            Final_Brick_2_pos = DoBotSpawn + transl(0.532,0.1344,0) * trotx(pi)
            Initial_Brick_7_pos = transl(Brick7) * trotx(pi) %%replace transl w/ Brick 1
            Final_Brick_7_pos = DoBotSpawn+transl(0.266,0.1344,0.1334) * trotx(pi) %%replace transl w/ Brick 2

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
            Final_Brick_3_pos = DoBotSpawn + transl(0.800,0.1344,0) * trotx(pi)
            Initial_Brick_8_pos = transl(Brick8) * trotx(pi); %%replace transl w/ Brick 1
            Final_Brick_8_pos = DoBotSpawn+transl(0.532,0.1344,0.1334) * trotx(pi) %%replace transl w/ Brick 2

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
            Final_Brick_4_pos = DoBotSpawn+transl(0.266,0.1344,0.0667) * trotx(pi) %%replace transl w/ Brick 2
            Initial_Brick_9_pos = transl(Brick9) * trotx(pi) %%replace transl w/ Brick 1
            Final_Brick_9_pos = DoBotSpawn+transl(0.800,0.1344,0.1334) * trotx(pi) %%replace transl w/ Brick 2

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
            Final_Brick_5_pos = DoBotSpawn+transl(0.532,0.1344,0.0667) * trotx(pi) %%replace transl w/ Brick 2

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
        
        function Collision_detection(start)
            centerpnt = [-0.4,-1.35,0.1];
            side = 0.5;
            plotOptions.plotFaces = true;
            [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/4, centerpnt+side/2,plotOptions);
            axis equal
            camlight
            start.DoBot.teach;
            q = zeros(1,5);  
            tr = zeros(4,4,start.DoBot.n+1);
            tr(:,:,1) = start.DoBot.base;
            L = start.DoBot.links;
            for i = 1 : start.DoBot.n
            tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
            
            
            
        end % end of collision detection
        
        function Camera(start)
            hold on
            %Create Image target
            pStar = [10 1 1 10; 1 1 10 10];

            %Create 3D Points
            P=[1.8,1.8,1.8,1.8;
                -0.25,0.25,0.25,-0.25;
                1.25,1.25,0.75,0.75];



            %Add the camera
            cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
                'resolution', [1024 1024], 'centre', [512 512],'name', 'DoBot');

            %Frame Rate
            fps = 25;

            %Define values - gain of the controller
            lambda = 0.6;

            %Depth of IBVS
            depth = mean (P(1,:));

            r = start.DoBot();
            DoBotq_i = [0 0 0 0];

            Tc0 = start.DoBot.fkine(DoBotq_i);
            start.DoBot.animate(DoBotq_i);
            drawnow

            cam.T=Tc0

            % Display points in 3D and the camera
            cam.plot_camera('Tcam',Tc0, 'label','scale',0.05);
            hold on
            plot_sphere(P, 0.05, 'b');
            lighting gouraud
            light


            %Project points to the image
            p = cam.plot(P, 'Tcam', Tc0);
            pause(20)


            %
            %             %Camera view and plotting
            %             cam.clf()
            %             cam.plot(pStar, '*'); % create the camera view
            %             cam.hold(true);
            %             cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
            %             pause(2)
            %             cam.hold(true);
            %             cam.plot(P);    % show initial view
            %
            %             %Initialise display arrays
            %             vel_p = [];
            %             uv_p = [];
            %             history = [];
            %             hold on
            % loop of the visual servoing
            %             ksteps = 0;
            %             while true
            %               ksteps = ksteps + 1;
            %
            %              % compute the view of the camera
            %              uv = cam.plot(P);
            %
            %         % compute image plane error as a column
            %         e = pStar-uv;   % feature error
            %         e = e(:);
            %         Zest = [];
            %
            %         % compute the Jacobian
            %         if isempty(depth)
            %             % exact depth from simulation (not possible in practice)
            %             pt = homtrans(inv(Tcam), P);
            %             J = cam.visjac_p(uv, pt(3,:) );
            %         elseif ~isempty(Zest)
            %             J = cam.visjac_p(uv, Zest);
            %         else
            %             J = cam.visjac_p(uv, depth );
            %         end
            %
            %         % compute the velocity of camera in camera frame
            %         try
            %             v = lambda * pinv(J) * e;
            %         catch
            %             status = -1;
            %             return
            %         end
            %         fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
            %
            %         %compute robot's Jacobian and inverse
            %         J2 = start.DoBot.jacobn(DoBotq_i);
            %         Jinv = pinv(J2);
            %         % get joint velocities
            %         qp = Jinv*v;
            %
            %
            %          %Maximum angular velocity cannot exceed 180 degrees/s
            %          ind=find(qp>pi);
            %          if ~isempty(ind)
            %              qp(ind)=pi;
            %          end
            %          ind=find(qp<-pi);
            %          if ~isempty(ind)
            %              qp(ind)=-pi;
            %          end
            %
            %         %Update joints
            %         q = DoBotq_i + (1/fps)*qp;
            %         start.DoBot.animate(q');
            %
            %         %Get camera location
            %         Tc = start.DoBot.fkine(q);
            %         cam.T = Tc;
            %
            %         drawnow
            %
            %         % update the history variables
            %         hist.uv = uv(:);
            %         vel = v;
            %         hist.vel = vel;
            %         hist.e = e;
            %         hist.en = norm(e);
            %         hist.jcond = cond(J);
            %         hist.Tcam = Tc;
            %         hist.vel_p = vel;
            %         hist.uv_p = uv;
            %         hist.qp = qp;
            %         hist.q = q;
            %
            %         history = [history hist];
            %
            %          pause(1/fps)
            %
            %         if ~isempty(200) && (ksteps > 200)
            %             break;
            %         end
            %
            %         %update current joint position
            %         DoBotq_i = q;
            %          end %loop finishes
            %
            %
            %         figure()
            %         plot_p(history,pStar,cam)
            %         figure()
            %         plot_camera(history)
            %         figure()
            %         plot_vel(history)
            %         figure()
            %         plot_robjointpos(history)
            %         figure()
            %         plot_robjointvel(history)
            %
            %         % image plane trajectory
            %             uv = [history.uv]';
            %             % result is a vector with row per time step, each row is u1, v1, u2, v2 ...
            %             for i=1:numcols(uv)/2
            %                 p = uv(:,i*2-1:i*2);    % get data for i'th point
            %                 plot(p(:,1), p(:,2))
            %             end
            %             plot_poly( reshape(uv(1,:), 2, []), 'o--');
            %             uv(end,:)
            %             if ~isempty(uv_star)
            %                 plot_poly(uv_star, '*:')
            %             else
            %                 plot_poly( reshape(uv(end,:), 2, []), 'rd--');
            %             end
            %             axis([0 camera.npix(1) 0 camera.npix(2)]);
            %             set(gca, 'Ydir' , 'reverse');
            %             grid
            %             xlabel('u (pixels)');
            %             ylabel('v (pixels)');
            %             hold off
            %
            %             vel = [history.vel]';
            %             plot(vel(:,1:3), '-')
            %             hold on
            %             plot(vel(:,4:6), '--')
            %             hold off
            %             ylabel('Cartesian velocity')
            %             grid
            %             xlabel('Time')
            %             xaxis(length(history));
            %             legend('v_x', 'v_y', 'v_z', '\omega_x', '\omega_y', '\omega_z')
        end
        function Collisions(start)
            pause(0.1)
            
            radius = 0.5;
            sphereCenter = [1,1,0];
            tr = start.DoBot.fkine(start.DoBot.getpos);
            EEtoCentre = sqrt(sum((start.rb_trans-tr(1:3,4)').^2)); %%Replace rb_trans       
              if EEtoCentre <= radius %% End Effector Distance
             disp('Oh no a collision!');
%            isCollision = 1;
              else
             disp(['SAFE: End effector to sphere centre distance (', num2str(EEtoCentre), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
%         isCollision = 0;
            end

        end
        function Main(start)
            pause(3)
            robot = DoBot2; %% Enable DoBot
            robot.model.base = start.DoBotSpawn;

            pause(3)
            i = 1;
            x = 0.1;
            while start.running == true
                
                j=1;
                while j <=2 % test moving all bricks at the same time
                    start.rb_trans(:,:,j) = transl(-0.65+j*0.25,-1+(i*0.003),0)*trotz(0.02*i);
                    start.gb_trans(:,:,j) = transl(-0.65+j*0.25,-1.5+(i*0.003),0)*trotz(0.02*i);
                    start.bb_trans(:,:,j) = transl(-0.65+j*0.25,-2+(i*0.003),0)*trotz(0.02*i);
                    j=j+1;
                end
                j=1;
                while j <=2 % each loop of main, put each brick where its transform array says it should be (animation purposes)
                    start.RedtransVert = [start.rb_vert(:,:,j),ones(size(start.rb_vert(:,:,j),1),1)] * start.rb_trans(:,:,j)';
                    start.GreentransVert = [start.gb_vert(:,:,j),ones(size(start.gb_vert(:,:,j),1),1)] * start.gb_trans(:,:,j)';
                    start.BluetransVert = [start.bb_vert(:,:,j),ones(size(start.bb_vert(:,:,j),1),1)] * start.bb_trans(:,:,j)';

                    set(start.rbricks(j),'Vertices',start.RedtransVert(:,1:3));
                    set(start.gbricks(j),'Vertices',start.GreentransVert(:,1:3));
                    set(start.bbricks(j),'Vertices',start.BluetransVert(:,1:3));


                    j=j+1;
                end
                drawnow()
                
                i = i + 1;
%                 start.DoBot.fkine(DoBotq_i); %% For Camera sensor.. unsure if needed here

                robot.model.animate([x,0,0,0,0]);
                x=x+0.05;
            end % end of animation loop

        end %end of main
   

       end
    end
  


