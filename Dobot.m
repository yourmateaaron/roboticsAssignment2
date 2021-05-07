classdef Dobot < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-1 1 -1 1 -0.3 1];
        scale = 0.5;
    end
    
    methods%% Class for Dobot robot simulation
function self = Dobot()
    
% robot = 
self.GetDobotRobot();
% robot = 
self.PlotRobot();
% self.PlotAndColourRobot();%robot,workspace);

end

%% GetDobotRobot
% Given a name (optional), create and return a Dobot robot model
function GetDobotRobot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['Dobot_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

    % DH Parameters
    L(1) = Link('d',0.138,  'a',0,      'alpha',-pi/2, 'offset',0);
    L(2) = Link('d',0,      'a',0.135,  'alpha',0,     'offset',-pi/2);
    L(3) = Link('d',0,      'a',0.147,  'alpha',0,     'offset',0);
    L(4) = Link('d',0,      'a',0.035,  'alpha',pi/2,  'offset',-pi/2);     
    L(5) = Link('d',0,      'a',0,      'alpha',0,     'offset',0);

    % Joint Limits
    L(1).qlim = [-135 135]*pi/180;
    L(2).qlim = [-5 85]*pi/180;
    L(3).qlim = [-5 85]*pi/180;
    L(4).qlim = [-90 90]*pi/180;
    L(5).qlim = [-90 90]*pi/180;

    self.model = SerialLink(L,'name',name);
end
%% Plot
function PlotRobot(self)
    self.model.plot(zeros(1,self.model.n),'workspace',self.workspace,'scale',self.scale)
    view(3);
end

%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
% function PlotAndColourRobot(self)%robot,workspace)
%         [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
%         self.model.faces{linkIndex+1} = faceData;
%         self.model.points{linkIndex+1} = vertexData;

%     % Display robot
%     self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
%     if isempty(findobj(get(gca,'Children'),'Type','Light'))
%         camlight
%     end  
%     self.model.delay = 0;
% 
%     % Try to correctly colour the arm (if colours are in ply file data)
%     for linkIndex = 0:self.model.n
%         handles = findobj('Tag', self.model.name);
%         h = get(handles,'UserData');
%         try 
%             h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
%                                                           , plyData{linkIndex+1}.vertex.green ...
%                                                           , plyData{linkIndex+1}.vertex.blue]/255;
%             h.link(linkIndex+1).Children.FaceColor = 'interp';
%         catch ME_1
%             disp(ME_1);
%             continue;
%         end
%     end        
end
end