classdef Simulator2 < handle
    % This version is used to generate the validation data set for section
    % 4.3
    properties
        hFigure
        HD
        
        hResultsWindow
        aVars       % Algorithm Variables
        
        Player
        
        SceneObjects
        StaticObjects
        MovingObjects
        
        MakeVideo = true;
%         MakeVideo = false;
        
        Lighting
    end
    
    properties(Constant)
        mi2m = 1609.34;     % Miles to Meters
        
        SimulationDistance = 1 * 1609.34; % 1 mile
        SimulationParams = struct('numCars', 4);

        RoadParams = struct('numLanes', 3,...
                            'laneWidth', 3,... %m
                            'laneMarkerWidth', 0.127,... %m (6in)
                            'roadColor', [0.1 0.1 0.1],...
                            'laneColor', [1 1 1]);

        DriverParams = struct('lookAheadDistance', 0.5 * 1609.34); %m
        
        CarParams = struct('length', 4.5,...    %m
                           'width', 1.7,...     %m
                           'height', 1.25,...   %m
                           'color', [0.5 0 0]);
            
        CameraParams = struct('height', 1);
    end
    
    methods
        function self = Simulator2(varargin)
            self.hFigure = clf(figure(1));
            
            set(self.hFigure, 'Name', 'Perspective Simulator',...
                              ...'Units', 'normalized',...
                              ...'Position', [0.0 0.0 0.75 0.75],...
                              'Visible', 'off',...
                              'Renderer', 'opengl',...
                              'defaultSurfaceEdgeColor', 'none',...
                              'defaultPatchEdgeColor', 'none');
            
            self.MakeWindow();
            
            rng(26)
            
            self.CreateScene()
            self.CreateStaticObjects();
            self.CreateMovingObjects();
            self.CreateLighting();
            
            %% Set the initial camara pos
            posvec = [0 self.RoadParams.laneWidth/2 self.CameraParams.height];
            campos(posvec);
            targvec = posvec + [self.DriverParams.lookAheadDistance 0 0];
            camtarget(targvec);  % The camera looks along the +Z axis
%             vertvec = [0 0 1];
%             camup(vertvec/norm(vertvec))
            
            camproj('perspective')
            if verLessThan('matlab', '8.5')
                camva(10)
%                 % Why do we need to do this???? And it still doesn't fully work
%                 set(gca, 'xlim', [-10 500], 'ylim', [-10000 10000], 'zlim', [-10000, 10000]);   
            else
%                 camva(45)
                camva(20)
%                 camva(60)
            end

            %% Make the GUI ready for viewing
            movegui(self.hFigure, 'center')
            set(self.hFigure, 'Visible', 'on');
            
            %% Start the simulation
            self.InitializeAlgorithm();
            
            if nargin > 0
                return
            end
            self.Simulate();
        end
        
        function MakeWindow(self)
            self.HD.MainView = axes('Parent', self.hFigure');
            
%             imsize = [1 1 512 512];
            imsize = [1 1 640 480];
%             imsize = [1 1 800 600];
%             imsize = [1 1 1024 768];
%             imsize = [1 1 1280 960];
            
            self.aVars.imsize = imsize(3:4);
            
            set(self.HD.MainView,...
                'Color', 'c',...
                'ydir', 'reverse',...    % this it to make it NED-ish, we'll treat Z as up
                'Units', 'pixels',....
                ...'PlotBoxAspectRatio', [1 1 1],...
                'Position', imsize); % This make it the desired image size
            set(self.hFigure, 'Units', 'pixels',...
                'Position', imsize)
            
            axis(self.HD.MainView, 'image');
%             axis(self.HD.MainView, 'off');

            xlabel('Down Range (m)');
            ylabel('Cross Range (m)');
            zlabel('Altitude (m)');
            
            %% Add menu options
        end
        
        function CreateScene(self)
            %% Draw the ground plane
            [x,y,z] = self.Ground(self.SimulationDistance);
            self.SceneObjects.Ground = patch(x, y, z, [0 0.5 0], 'UserData', [0 0.5 0]);
            
            %% Draw the background plane
            [x,y,z] = self.Background(self.SimulationDistance);
            self.SceneObjects.Background = patch(x, y, z, [0 0.5 0.75], 'UserData', [0 0.5 0.75]);
            
            %% Draw the roadway
            [x,y,z] = self.Road(self.SimulationDistance, self.RoadParams);
            self.SceneObjects.Road = patch(x, y, z, self.RoadParams.roadColor, 'UserData', self.RoadParams.roadColor);
            self.SceneObjects.OpposingRoad = patch(x, -y, z, self.RoadParams.roadColor, 'UserData', self.RoadParams.roadColor);
            
            %% Draw the lane markings
            % Double yellow centerline Markings
            [x,y,z] = self.SolidLaneMarkings(self.SimulationDistance, self.RoadParams);
            self.SceneObjects.LaneMarkings(1) = patch(x, y+.25*self.RoadParams.laneMarkerWidth, z, [1 1 0], 'UserData', [1 1 0]);
            self.SceneObjects.LaneMarkings(end+1) = patch(x, -y-.25*self.RoadParams.laneMarkerWidth, z, [1 1 0], 'UserData', [1 1 0]);
            
            % Dashed white lines
            [x,y,z] = self.DashedLaneMarkings(self.SimulationDistance, self.RoadParams);
            for n = 1:(self.RoadParams.numLanes-1)
                self.SceneObjects.LaneMarkings(end+1) = patch(x, y + n*self.RoadParams.laneWidth - self.RoadParams.laneMarkerWidth/2, z, self.RoadParams.laneColor, 'UserData', self.RoadParams.laneColor);
                self.SceneObjects.LaneMarkings(end+1) = patch(x, -(y + n*self.RoadParams.laneWidth - self.RoadParams.laneMarkerWidth/2), z, self.RoadParams.laneColor, 'UserData', self.RoadParams.laneColor);
            end
            
            % Solid white edge lines
            [x,y,z] = self.SolidLaneMarkings(self.SimulationDistance, self.RoadParams);
            self.SceneObjects.LaneMarkings(end+1) = patch(x, y + (n+1)*self.RoadParams.laneWidth - self.RoadParams.laneMarkerWidth/2, z, self.RoadParams.laneColor, 'UserData', self.RoadParams.laneColor);
            self.SceneObjects.LaneMarkings(end+1) = patch(x, -(y + (n+1)*self.RoadParams.laneWidth - self.RoadParams.laneMarkerWidth/2), z, self.RoadParams.laneColor, 'UserData', self.RoadParams.laneColor);
            
        end
        
        function CreateStaticObjects(self)
            %% Create a row of trees alongside the road
            y = (self.RoadParams.numLanes + 0.5) * self.RoadParams.laneWidth + 4;
            treeSeperation = 100; %m
            numTrees = floor(self.SimulationDistance/treeSeperation);      % Place a tree every 10 meters
            for n = 1:numTrees
                self.StaticObjects.Trees(n) = self.Tree(n*treeSeperation, y);
                self.StaticObjects.Trees(n+numTrees) = self.Tree(n*treeSeperation, -y);
            end
        end
        
        function CreateMovingObjects(self)
            %% Draw some "Cars"
            [x,y,z] = self.Car(self.CarParams);
%             carlane = randi(self.RoadParams.numLanes,1,self.SimulationParams.numCars)-1;
            carlane = [2 0 2 0];
            for n = 1:self.SimulationParams.numCars
                self.MovingObjects.Cars(n) = surface(x + 10 + 25*n, y + (carlane(n)+0.5)*self.RoadParams.laneWidth - self.CarParams.width/2, z, 'FaceColor', self.CarParams.color, 'UserData', self.CarParams.color);
            end
            
        end
        
        function CreateLighting(self)
            count = 1;
            for n = 0:500:self.SimulationDistance
                self.Lighting(count) = light('Position', [n, 0 -2000]);
                count = count+1;
            end
        end
    end
    
    methods
        
        function InitializeAlgorithm(self)
            self.hResultsWindow = figure;
            
            %% Initialize the Monte-Carlo Vanishing Point Tracker(MCVPT)
            self.aVars.VPTracker = VPTracker(1000, self.aVars.imsize);
            
            %% Initialize the Inverse Perspective Mapping (IPM) class
            if true
                savedIPM = load('@IPM\myIPM.mat');
                self.aVars.IPM = savedIPM.myIPM;
            else
                myIPM = IPM(fliplr(self.aVars.imsize),...
                    'cameraZ', 4.4,...
                    'theta', atan(self.CameraParams.height/self.DriverParams.lookAheadDistance),...
                    'stepSize', [.125 .25],...
                    'xRange', [-34 34],...
                    'yRange', [0 300]);  % 328.084]);
                self.aVars.IPM = myIPM;
                save('myIPM.mat', 'myIPM');
            end
            
            self.aVars.sampleRegionI = 400:450; %->
            self.aVars.sampleRegionJ = 270:370; %v
            self.aVars.intensityRange = 35;
            
        end
        
        function Simulate(self)
            delT = 1/8;
            if self.MakeVideo
                mov = VideoWriter('Examples\CamSeqManipulation\SimulatedDataSet.avi');
                mov.FrameRate = round(1/delT);
                open(mov);
                
                movGT = VideoWriter('Examples\CamSeqManipulation\SimulatedDataSetGT.avi');
                movGT.FrameRate = round(1/delT);
                open(movGT);
            end
            
            % For a car traveling at 20m/s (~45mph) it will take ~80sec to
            % travel 1 mile
            tic
            posvec = [10 3*self.RoadParams.laneWidth/2 self.CameraParams.height];
            commandedHeading = 0;
            currentYaw = 0;
            commandedSpeed = 29; % Travel @ 65 mph
            currentSpeed = commandedSpeed;
            for t = 0:delT:20% 34.25;%
                if ~ishandle(self.HD.MainView)
                    break
                end
                %% Update the vehicle/dynamics/scene simulation
                figure(self.hFigure);
                headingError = (commandedHeading-currentYaw);
                speedError = commandedSpeed - currentSpeed;
                currentYaw = currentYaw + sign(headingError)*min(abs(headingError), 20*pi/180)*delT;
                currentSpeed = currentSpeed + 0.5*speedError;
                
                posvec = posvec + currentSpeed*[cos(currentYaw) sin(currentYaw) 0]*delT; 
                targvec = posvec + self.DriverParams.lookAheadDistance*[cos(currentYaw) sin(currentYaw) 0];
                
                campos(posvec);
                camtarget(targvec);
                
                % Move the cars
                for m = 1:self.SimulationParams.numCars
                    xpos = get(self.MovingObjects.Cars(m), 'XData');
                    set(self.MovingObjects.Cars(m), 'XData', xpos + 26*delT)
                end
                
                %% Grab the image
                rgb = getframe(self.HD.MainView);
                rgb = rgb.cdata;
                
                set(self.Lighting, 'Visible', 'off')
                rgbGT= getframe(self.HD.MainView);
                rgbGT = rgb2ind(rgbGT.cdata, 256);
                
                set(self.Lighting, 'Visible', 'on')
                %% Update the results display
                figure(self.hResultsWindow);
                ax = gca(self.hResultsWindow);
                cla(ax)
                
                imshow(rot90(rgb,2), 'Parent', ax), hold(ax, 'on')
                title(sprintf('Time %0.2f', t))
                axis equal tight on
                set(ax,'yDir','normal', 'xdir','reverse')
                
                
                %% Store the results window in a video
                if self.MakeVideo
                    im = getframe(ax);
                    writeVideo(mov, im.cdata);
                    
                    writeVideo(movGT, rgbGT);
                end                
                drawnow
            end
            if self.MakeVideo
                close(mov)
            end
            toc
        end
    end
    
    methods(Static)
        function [x,y,z] = Ground(length)
            x = [-1 length length -10];
            y = [-100 -100 100 100];
            z = [0 0 0 0];
        end
        
        function [x,y,z] = Background(length)
            x = length*[1 1 1 1];
            y = 10000*[-1 1 1 -1];
            z = 10000*[-1 -1 1 1];
        end
        
        function [x,y,z] = Road(length, RoadParams)
            width = (RoadParams.numLanes + 0.5) * RoadParams.laneWidth;
            x = [0 0 length length];
            y = [0 width width 0];
            z = [0 0 0 0] + 0.01;
        end
        
        function [x,y,z] = DoubleLaneMarkings(length, RoadParams)
            width = RoadParams.laneMarkerWidth;
            x = [0 0 length length 0 0 0 length length 0];
            y = width*[-0.25 -1.25 -1.25 -0.25 -0.25 0.25 1.25 1.25 0.25 0.25];
            z = [0 0 0 0 0 0 0 0 0 0] + 0.02;
            
%             width = RoadParams.laneMarkerWidth;
%             x = [0 0 length length];
%             y = [0 width width 0];
%             z = [0 0 0 0] + 0.02;
        end
        
        function [x,y,z] = SolidLaneMarkings(length, RoadParams)
            width = RoadParams.laneMarkerWidth;
            x = [0 0 length length];
            y = [0 width width 0];
            z = [0 0 0 0] + 0.02;
        end
        
        function [x,y,z] = DashedLaneMarkings(length, RoadParams)
            % On rural highways, broken lines should consist of 3 m (10 ft)
            % line segments and 9 m (30 ft) gaps, or similar dimensions in
            % a similar ratio of line segments to gaps as appropriate for
            % traffic speeds and need for delineation
            %
            % Option: A dotted line may consist of 0.6m (2ft) line
            % segments, and 1.2m (4ft) or longer gaps, with a maximum
            % segment to gap ratio of 1-to-3 
            % Ref. http://mutcd.fhwa.dot.gov/pdfs/millennium/06.14.01/3ndi.pdf
            % page 10
            
            width = RoadParams.laneMarkerWidth;
            segLen = 3;
            origx = [0 0 segLen segLen];
            origy = [0 width width 0];
            origz = [0 0 0 0] + 0.02;
            
            seperation = segLen + segLen*3;
            numSegments = floor(length/(seperation)); 
            x = [];
            for n = 1:numSegments
                x = [x origx+n*seperation];
            end
            y = repmat(origy,1,numSegments);
            z = repmat(origz,1,numSegments);
        end
        
        function [x,y,z] = Car(CarParams)
            x = CarParams.length*[0.5 0.5 0.5 0.5 0.5; 0 0 1 1 0; 0 0 1 1 0; 0.5 0.5 0.5 0.5 0.5];
            y = CarParams.width *[0.5 0.5 0.5 0.5 0.5; 0 1 1 0 0; 0 1 1 0 0; 0.5 0.5 0.5 0.5 0.5];
            z = CarParams.height*[0 0 0 0 0; 0 0 0 0 0; 1 1 1 1 1; 1 1 1 1 1] + 3*0.01;
        end
        
        function h = Tree(x,y)
            ang = linspace(0,2*pi,10);
            bx = cos(ang);
            by = sin(ang);
            bz = ones(size(ang));
            trunkx = [0*bx;
                      0.3048*bx;    % 1ft
                      0.3048*bx;    % 1ft
                      0*bx];
            trunky = [0*by;
                      0.3048*by;    % 1ft
                      0.3048*by;    % 1ft
                      0*by];
            trunkz = [0*bz;
                      0*bz;         
                      0.9144*bz;    % 3ft
                      0.9144*bz];   % 3ft
                
            topx = [0*bx;
                    1.8288*bx;    % 6ft
                    0*bx];
            topy = [0*by;
                    1.8288*by;    % 6ft
                    0*by];
            topz = [0.9144*bz;   % 3ft
                    0.9144*bz;   % 3ft      
                    4.572*bz];   % 15ft
                  
            h.trunk = surface(trunkx + x, trunky + y, trunkz, 'FaceColor', [92 64 51]/255, 'UserData', [92 64 51]/255);
            h.top = surface(topx + x, topy + y, topz, 'FaceColor', [25 50 25]/255, 'UserData', [25 50 25]/255);
        end
        
        % Color change static properties
        function MakeBlack(h)
            try
                set(h, 'Color', [0 0 0]);
            catch
                set(h, 'FaceColor', [0 0 0]);
            end
        end
        
        function RestoreColor(h)
            color = get(h, 'UserData');
            try
                set(h, 'Color', color);
            catch
                if length(h) > 1
                    for n = 1:length(h)
                        set(h(n), 'FaceColor', color{n})
                    end
                else
                    set(h, 'FaceColor', color);
                end
            end
        end
        
        function [minv, maxv] = minmax(x)
            minv = min(x(:));
            maxv = max(x(:));
        end
        
        function y = wraprad(x)
            % Helper function to wrap radians to be between -pi & pi 
            y = x - sign(x).*fix((abs(x)+pi)/(2*pi))*2*pi;
        end
    end
end