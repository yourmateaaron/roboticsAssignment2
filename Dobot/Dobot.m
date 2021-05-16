classdef Dobot < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-3 3 -3 3 -1.1 2];
        scale = 0.5;
        qhome = [0    0.1166    1.4480    1.5770         0];
    end
    
    methods%% Class for Dobot robot simulation
function self = Dobot()
    
% robot = 
self.GetDobotRobot();
% robot = 
self.PlotAndColourRobot();
% self.PlotAndColourRobot();%robot,workspace);

end

%% GetDobotRobot
% Given a name (optional), create and return a Dobot robot model
function GetDobotRobot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = 'Wiper';
%     end

    % DH Parameters
    L(1) = Link('d',0.138,  'a',0,      'alpha',-pi/2, 'offset',0);
    L(2) = Link('d',0,      'a',0.135,  'alpha',0,     'offset',-pi/2);
    L(3) = Link('d',0,      'a',0.147,  'alpha',0,     'offset',0);
    L(4) = Link('d',0,      'a',0.044,  'alpha',pi/2,  'offset',-pi/2);     
    L(5) = Link('d',-0.138,      'a',0,      'alpha',0,     'offset',0);

    % Joint Limits
    L(1).qlim = [-135 135]*pi/180;
    L(2).qlim = [5 80]*pi/180;
    L(3).qlim = [15 170]*pi/180;
    L(4).qlim = [-90 90]*pi/180;
    L(5).qlim = [-85 85]*pi/180;
    
   

    self.model = SerialLink(L,'name',name);
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['DobotLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(self.qhome,'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
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
%% Determine the base of the robot and plot the 3D model
function DobotBaseAndPlot(self,position)
    self.model.base = transl(position);
    self.PlotAndColourRobot();
end
%% GetLinkPoses
% Gets the transform for every joint
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( self, q )
    
    transforms = zeros(4,4,self.model.n+1);
    transforms(:,:,1) = self.model.base;
    L = self.model.links;
    
    for i = 1:self.model.n
        transforms(:,:,i+1) = transforms(:,:,i) * trotz(q(i)+L(i).offset)...
                                                * transl(0,0,L(i).d)...
                                                * transl(L(i).a,0,0)...
                                                * trotx(L(i).alpha);
    end
end

end
end