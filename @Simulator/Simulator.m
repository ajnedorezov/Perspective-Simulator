classdef Simulator < handle
    properties
        hFigure
        HD
        
        hResultsWindow
        aVars       % Algorithm Variables
        
        Player
        
        SceneObjects
        StaticObjects
        MovingObjects
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
        function self = Simulator(varargin)
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
            
            self.Simulate();
        end
        
        function MakeWindow(self)
            self.HD.MainView = axes('Parent', self.hFigure');
            
%             imsize = [1 1 512 512];
            imsize = [1 1 640 480];
%             imsize = [1 1 800 600];
%             imsize = [1 1 1024 768];
%             imsize = [1 1 1280 960];
            
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
            self.SceneObjects.Ground = patch(x, y, z, [0 0.5 0]);
            
            %% Draw the background plane
            [x,y,z] = self.Background(self.SimulationDistance);
            self.SceneObjects.Background = patch(x, y, z, [0 0.5 0.75]);
            
            %% Draw the roadway
            [x,y,z] = self.Road(self.SimulationDistance, self.RoadParams);
            self.SceneObjects.Road = patch(x, y, z, self.RoadParams.roadColor);
            self.SceneObjects.OpposingRoad = patch(x, -y, z, self.RoadParams.roadColor);
            
            %% Draw the lane markings
            % Double yellow centerline Markings
            [x,y,z] = self.SolidLaneMarkings(self.SimulationDistance, self.RoadParams);
            self.SceneObjects.LaneMarkings(1) = patch(x, y+.25*self.RoadParams.laneMarkerWidth, z, [1 1 0]);
            self.SceneObjects.LaneMarkings(end+1) = patch(x, -y-.25*self.RoadParams.laneMarkerWidth, z, [1 1 0]);
            
            % Dashed white lines
            [x,y,z] = self.DashedLaneMarkings(self.SimulationDistance, self.RoadParams);
            for n = 1:(self.RoadParams.numLanes-1)
                self.SceneObjects.LaneMarkings(end+1) = patch(x, y + n*self.RoadParams.laneWidth - self.RoadParams.laneMarkerWidth/2, z, self.RoadParams.laneColor);
                self.SceneObjects.LaneMarkings(end+1) = patch(x, -(y + n*self.RoadParams.laneWidth - self.RoadParams.laneMarkerWidth/2), z, self.RoadParams.laneColor);
            end
            
            % Solid white edge lines
            [x,y,z] = self.SolidLaneMarkings(self.SimulationDistance, self.RoadParams);
            self.SceneObjects.LaneMarkings(end+1) = patch(x, y + (n+1)*self.RoadParams.laneWidth - self.RoadParams.laneMarkerWidth/2, z, self.RoadParams.laneColor);
            self.SceneObjects.LaneMarkings(end+1) = patch(x, -(y + (n+1)*self.RoadParams.laneWidth - self.RoadParams.laneMarkerWidth/2), z, self.RoadParams.laneColor);
            
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
            carlane = randi(self.RoadParams.numLanes,1,self.SimulationParams.numCars)-1;
%             carlane = [2 0 1 1];
            for n = 1:self.SimulationParams.numCars
                self.MovingObjects.Cars(n) = surface(x + 25*n, y + (carlane(n)+0.5)*self.RoadParams.laneWidth - self.CarParams.width/2, z, 'FaceColor', self.CarParams.color);
            end
            
        end
        
        function CreateLighting(self)
            for n = 0:500:self.SimulationDistance
                light('Position', [n, 0 -2000])
            end
        end
    end
    
    methods
        
        function InitializeAlgorithm(self)
            self.hResultsWindow = figure;
            
        end
        
        
        function Simulate(self)
            
%             mov = VideoWriter('temp.avi');
%             open(mov);
            
            % For a car traveling at 20m/s (~45mph) it will take ~80sec to
            % travel 1 mile
            % 
            delT = 1/8;
            initializeSP = true;
            for t = 15%:delT:80
                if ~ishandle(self.HD.MainView)
                    break
                end
                %% Update the vehicle/dynamics/scene simulation
                figure(self.hFigure);
                posvec = [t self.RoadParams.laneWidth*sin(t/25) + 3*self.RoadParams.laneWidth/2 self.CameraParams.height];
                targvec = posvec + self.DriverParams.lookAheadDistance*[1 self.RoadParams.laneWidth/25*cos(t/25) 0];
                campos(posvec);
                camtarget(targvec);

%                 im = getframe(self.HD.MainView);
%                 
%                 writeVideo(mov, im.cdata);
                pause(0.1), drawnow
                
                if initializeSP
                    initializeSP = false;
                    
                    %% Initialize SP
                    rgb = getframe(self.HD.MainView);
                    rgb = rgb.cdata;
                    imsize = size(rgb);
                    self.aVars.imsize = imsize;

                    self.aVars.k = 400;                                             % # of superpixels
                    self.aVars.N = prod(imsize(1:2));                    % # of pixels
                    self.aVars.S = sqrt(self.aVars.N/self.aVars.k);                 % # pixels / superpixel

                    self.aVars.numStepsHeight = ceil(imsize(1)/self.aVars.S);
                    self.aVars.stepHeight = imsize(1)/self.aVars.numStepsHeight;

                    self.aVars.numStepsWidth = ceil(imsize(2)/self.aVars.S);
                    self.aVars.stepWidth = imsize(2)/self.aVars.numStepsWidth;

                    self.aVars.heightSpacing = round(((1:self.aVars.numStepsHeight)-0.5)*self.aVars.stepHeight);
                    self.aVars.widthSpacing = round(((1:self.aVars.numStepsWidth)-0.5)*self.aVars.stepWidth);

%                     [sr, sc] = meshgrid(self.aVars.heightSpacing, self.aVars.widthSpacing);
%                     [imc, imr] = meshgrid(1:imsize(2), 1:imsize(1));

                    cform = makecform('srgb2lab');
                    lab = double(applycform(rgb,cform));

                    clustNum = 1;
                    for r = self.aVars.heightSpacing
                        for c = self.aVars.widthSpacing
                            self.aVars.clust(clustNum) = Pixel(clustNum,r,c,lab(r,c,1:3), self.aVars.S*[1 1]);
                            clustNum = clustNum + 1;
                        end
                    end
                    self.aVars.numClusters = clustNum - 1;

                    %% Initialize the assignment variables and other intermediate variables
                    self.aVars.labels = zeros(imsize(1:2));
                    self.aVars.distance = inf(imsize(1:2));
                    self.aVars.E = 0;
                    self.aVars.startingEnergy = inf;
                    self.aVars.energy = 0;
                    self.aVars.previousEnergy = inf;
                end
                
                %% Grab the image and convert to lab
                rgb = getframe(self.HD.MainView);
                rgb = rgb.cdata;
                cform = makecform('srgb2lab');
                lab = double(applycform(rgb,cform));
                
                %% Perform the Super Pixel Segmentation
                for m = 1:10
                    % Loop over each cluster center Ck
                    for p = 1:self.aVars.numClusters
                        dmat = self.aVars.clust(p).calcDistanceMatrix(lab);
                        ind = dmat < self.aVars.distance;
                        self.aVars.distance(ind) = dmat(ind);
                        self.aVars.labels(ind) = self.aVars.clust(p).pixelNumber;        
                    end
                    energy = sum(self.aVars.distance(:));
                    fprintf('StartE = %g, PrevE = %g, CurrE = %g\n', self.aVars.startingEnergy, self.aVars.previousEnergy, energy)

                    % Compute the new residual error 
                    if isinf(self.aVars.startingEnergy)
                        self.aVars.startingEnergy = energy;
                    elseif ((self.aVars.previousEnergy - energy) < 1e-5 * (self.aVars.startingEnergy - energy))
                        break
                    end
                    self.aVars.previousEnergy = energy;

                    % Compute new cluster centers
                    for p = 1:self.aVars.numClusters
                        self.aVars.clust(p).Update(self.aVars.labels == p);
                    end
                end

                for m = 1:self.aVars.numClusters
                    self.aVars.clust(m).UpdateHistory();
                end
                
                %% Display the segmentation
                
                newlabels = self.aVars.labels;
                mask = imdilate(newlabels,ones(3,3)) > imerode(newlabels,ones(3,3));

                r = rgb(:,:,1);
                g = rgb(:,:,2);
                b = rgb(:,:,3);
                r(mask) = 255;
                g(mask) = 255;
                b(mask) = 255;
                partialIm = cat(3,r,g,b);

                figure(self.hResultsWindow);
                cla
                imshow(uint8(partialIm)); hold on,
                title(sprintf('Time %0.2f', t))
                
                
            end
%             close(mov)
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
                  
            h.trunk = surface(trunkx + x, trunky + y, trunkz, 'FaceColor', [92 64 51]/255);
            h.top = surface(topx + x, topy + y, topz, 'FaceColor', [25 50 25]/255);
        end
    end
    
    
end