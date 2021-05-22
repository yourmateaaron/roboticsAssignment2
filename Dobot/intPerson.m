classdef intPerson < handle
    
    properties (Constant)
        maxHeight = 2;
    end

    properties
        %> Number of cows
        personCount = 1;
        
        %> A cell structure of \c brickCount brick models
        person;
    
 
        workspaceDimensions = [-2.5 2.5 -2.5 2.5 -2.2 2];
    end
    
    methods
        %% ...structors
        function self = intPerson(personCount)
            if 0 < nargin
                self.personCount = personCount;
            end

            % Create the required number of bricks
            for i = 1:self.personCount
                self.person{i} = self.GetPersonModel(['person',num2str(i)]);
                % spawn
                self.person{i}.base = transl(0,2.5,-1);
                 % Plot 3D model
                plot3d(self.person{i},0,'workspace',self.workspaceDimensions,'delay',0);
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end

%             axis equal
%             camlight;
        end
        
        function delete(self)
%             cla;
        end       
        
        %% PlotSingleRandomStep
        % Move each of the cows forward and rotate some rotate value around
        % the z axis
        function PlotSingleStep(self)
            for personIndex = 1:self.personCount
                % Move towards dobot
                self.person{personIndex}.base = self.person{personIndex}.base * se3(se2(0, -0.05, 0));
                animate(self.person{personIndex},0);                
            end
            % Do the drawing once for each interation for speed
            drawnow();
        end    

        %% TestPlotManyStep
        % Go through and plot many random walk steps
        function crossLightCurtain(self,numSteps,delay)
            if nargin < 3
                delay = 0;
                if nargin < 2
                    numSteps = 28;
                end
            end
            for i = 1:numSteps
                self.PlotSingleStep();
                pause(delay);
            end
        end
    end
    
    methods (Static)
        %% GetpersonModel
        function model = GetPersonModel(name)
            if nargin < 1
                name = 'person';
            end
            [faceData,vertexData] = plyread('person.ply','tri');
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,1) = vertexData(:,1);
            vertexData(:,2) = vertexData(:,2);
            model.points = {vertexData,[]};
        end
    end    
end
