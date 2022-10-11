classdef DoBot < handle
    properties
        
        model;
        space = [-0.4 0.4 -0.4 0.4 -0.4 0.4];
        gripper = false;
    end
    
    methods
        function self = DoBot(gripper)
        if nargin<1
            gripper = false;
        end
        self.gripper = gripper;
        self.SetDoBot();
%         self.translatedo();
        end
    
        function SetDoBot(self);
        pause(0.001);
name = 'DoBOt magician';
            L1 = Link('d', 0.09, 'a', 0, 'alpha', pi/2, 'qlim', [-3*pi/4, 3*pi/4]);
            L2 = Link('offset',deg2rad(-32.5),'d', 0,'a', -0.152, 'alpha', 0, 'qlim', [0, 17*pi/36]);
            L3 = Link('offset',deg2rad(62.25),'d', 0, 'a',-0.147, 'alpha', 0, 'qlim', [-pi/18, 19*pi/36]);
            L4 = Link('offset',deg2rad(-30),'d', 0, 'a', -0.04, 'alpha', 0, 'qlim', [-pi/2, pi/2]);
            self.model = SerialLink([L1 L2 L3 L4],'name',name);
for linkno = 0:self.model.n
                [ faceData, vertexData, plyData{linkno+1} ] = plyread(['DOBOT_Magician-Link',num2str(linkno),'.ply'],'tri');
                self.model.faces{linkno+1} = faceData;
                self.model.points{linkno+1} = vertexData;
end
            
scale = 0.45;
 
            self.model.plotopt3d = {'wrist', 'xyz', 'arrow'};
            self.model.plot3d(zeros(1,self.model.n), 'workspace',self.space, 'scale', scale)
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
self.model.delay = 0;
            for linkno = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkno+1).Children.FaceVertexCData = [plyData{linkno+1}.vertex.red ...
                        , plyData{linkno+1}.vertex.green ...
                        , plyData{linkno+1}.vertex.blue]/255;
                    h.link(linkno+1).Children.FaceColor = 'interp';
                catch ME_1
                    continue;
                end
            end
        end
        function translatedo(self)
                        steps =100;
                        modelq_i = [0 0 0 0];
            target = [-0.25,0.25,0];
            hold on
            plot3(-0.25,0.25,0,'d');
            position = transl(target) * trotx(pi); 
            modelikcon = self.model.ikcon(position,modelq_i);
            Trajmodel = jtraj(modelq_i,modelikcon,steps);
%             Trajmodel(:,4) =0;
                        for i = 1:steps
                animate(self.model,Trajmodel(i,:))
                drawnow();
                pause(0.01)
                        end
        end
    end
end

%             steps =100;
%             target = [0.25,1.25,0];
%             position = transl(target) * trotx(pi); 
%             self.modelikcon = self.model.ikcon(position,self.modelq_i);
%             Trajself.model = jtraj(self.modelq_i,self.modelikcon,steps);
%             
%             
%                         for i = 1:steps
%                 animate(self.model,Trajself.model(i,:));
%                 drawnow();
%                 pause(0.01)
%                         end
%             