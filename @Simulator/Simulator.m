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
        
%         MakeVideo = true;
        MakeVideo = false;
        
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
            
            self.aVars.imsize = imsize([4 3]);
            
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
            carlane = randi(self.RoadParams.numLanes,1,self.SimulationParams.numCars)-1;
%             carlane = [2 0 1 1];
            for n = 1:self.SimulationParams.numCars
                self.MovingObjects.Cars(n) = surface(x + 25*n, y + (carlane(n)+0.5)*self.RoadParams.laneWidth - self.CarParams.width/2, z, 'FaceColor', self.CarParams.color, 'UserData', self.CarParams.color);
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
            
            imsize = self.aVars.imsize;
            
            %% Initilize the Monte-Carlo Vanishing Point Tracker(MCVPT)
            self.aVars.VPTracker = VPTracker(1000, imsize);
            
            %% Initialize some Birds-Eye-View (BEV) parameters
            alpha_tot = 30*pi/180;
            den = sqrt((imsize(2)-1)^2+(imsize(1)-1)^2);
            alpha = atan( (imsize(2)-1)/den * tan(alpha_tot) );
            self.aVars.BEVparams = struct('alpha', alpha,...
                                          'CameraLocationInWorld', [0 0 10],...
                                          'Gamma', 0,...
                                          'Theta', 0*pi/180,...atan(0.5/20),...
                                          'm', imsize(1),...
                                          'n', imsize(2));

            rHorizon = ceil((self.aVars.BEVparams.m-1)/(2*self.aVars.BEVparams.alpha)*(self.aVars.BEVparams.alpha-self.aVars.BEVparams.Theta)+1)+10;%-30;
            [v,u] = meshgrid((1:imsize(2))-1, (rHorizon):imsize(1));
            [X,Y] = self.ImageToWorld(v, u, self.aVars.BEVparams);

            step = 1;
            
            xg = fliplr(linspace(-30, 30, imsize(1)));% 34:-step:-26;% 
            yg = linspace(9, imsize(2)/2, imsize(2))';% (10:step:240)';% 

            self.aVars.BEV.rHorizon = rHorizon;
            self.aVars.BEV.X = X;
            self.aVars.BEV.Y = Y;
            self.aVars.BEV.step = step;
            self.aVars.BEV.xg = xg;
            self.aVars.BEV.yg = yg;
            
        end
        
        function Simulate(self)
            delT = 1/8;
            if self.MakeVideo
                mov = VideoWriter('Hough.avi');
                mov.FrameRate = round(1/delT);
                open(mov);
            end
            
            % For a car traveling at 20m/s (~45mph) it will take ~80sec to
            % travel 1 mile
            for t = 40%:delT:80
                if ~ishandle(self.HD.MainView)
                    break
                end
                %% Update the vehicle/dynamics/scene simulation
                figure(self.hFigure);
                posvec = [t self.RoadParams.laneWidth*sin(t/25) + 3*self.RoadParams.laneWidth/2 self.CameraParams.height];
                targvec = posvec + self.DriverParams.lookAheadDistance*[1 self.RoadParams.laneWidth/25*cos(t/25) 0];
                campos(posvec);
                camtarget(targvec);

                %% Grab the image
                rgb = getframe(self.HD.MainView);
                rgb = rgb.cdata;
                
                %% Update the VP Tracker
                control = [0 0];%[delT self.RoadParams.laneWidth*sin(t/25) + 3*self.RoadParams.laneWidth/2];
                self.aVars.VPTracker.Update(rgb, control);
                
%                 %% Do color thresholding to locate and classify the road/obstacles
% %                 % Turn off the lights to get the correct coloring
% %                 set(self.Lighting, 'Visible', 'off');
% %                 gtRGB = getframe(self.HD.MainView);
% %                 gtRGB = gtRGB.cdata;
% %                 
% %                 % Compare the image to the color labels
% %                 objectRGBIDs = [0 128 0;                    % Ground/Grass
% %                                 0 128 191;                  % Sky
% %                                 26 26 26;                   % Road
% %                                 255 255 255;                % White lane markers
% %                                 198 198 0;                  % Yellow lane markers
% %                                 128 0 0;                    % Car/obstacles
% %                                 [92 64 51];                 % Tree trunk
% %                                 [25 50 25];                 % Tree top
% %                                 ];
% %                             
% %                 labels = zeros(fliplr(self.aVars.imsize));
% %                 for n = 1:size(objectRGBIDs,1)
% %                     dC = bsxfun(@minus, double(gtRGB), reshape(objectRGBIDs(n,:), 1, 1, 3));
% %                     dist = sqrt(sum(dC.*dC, 3));
% %                     labels(dist < 10) = n;
% %                 end
% %
% %                 % Turn the lights back on
% %                 set(self.Lighting, 'Visible', 'on');
% 
%                 % Change all the colors to black and look for things not
%                 % black
%                  partOrder = ...
%                     {'self.SceneObjects.Ground'
%                      'self.SceneObjects.Background'
%                      'self.SceneObjects.Road'
%                      'self.SceneObjects.OpposingRoad'
%                      '[self.SceneObjects.LaneMarkings(:)]'
%                      'self.MovingObjects.Cars'
%                      '[self.StaticObjects.Trees(:).top]'
%                      '[self.StaticObjects.Trees(:).trunk]'
%                      };
% 
%                 for n = 1:length(partOrder)
%                     eval(sprintf('self.MakeBlack(%s);', partOrder{n}));
%                 end
%                 
%                 % Turn back on features to get the label mapping
%                 labels = zeros(fliplr(self.aVars.imsize));
%                
%                 try
%                     for n = 1:length(partOrder)
%                         eval(sprintf('self.RestoreColor(%s);', partOrder{n}));
%                         gtRGB = getframe(self.HD.MainView);
%                         labels(rgb2gray(gtRGB.cdata)>3) = n;
%                         eval(sprintf('self.MakeBlack(%s);', partOrder{n}));  
%                     end
%                     for n = 1:length(partOrder)
%                         eval(sprintf('self.RestoreColor(%s);', partOrder{n}));
%                     end
%                 end
%                 
                %% Apply the BEV transform
                im = rgb;
%                 im = rgb2gray(rgb);
%                 im = labels;

                % Performe the BEV transform
                imBEV = zeros(length(self.aVars.BEV.yg), length(self.aVars.BEV.xg), size(im,3));
                [xx,yy] = meshgrid(self.aVars.BEV.xg, self.aVars.BEV.yg);
                for d = 1:size(im,3);
            %         imBEV(:,:,d) = interp2(u, v, double(im(:,:,d)), X, Y);
%                     tempIm = griddata(self.aVars.BEV.X, self.aVars.BEV.Y, double(im(self.aVars.BEV.rHorizon:end, :, d))/255, self.aVars.BEV.xg, self.aVars.BEV.yg, 'linear');
                    
                    interpIm = double(im(self.aVars.BEV.rHorizon:end, :, d))/255;
                    F = TriScatteredInterp(self.aVars.BEV.X(:), self.aVars.BEV.Y(:), interpIm(:));
                    imBEV(:,:,d) = rot90(F(xx,yy),2);
                end
                imBEV = uint8(imBEV*255);

%                 imBEV = griddata(self.aVars.BEV.X, self.aVars.BEV.Y, double(im(self.aVars.BEV.rHorizon:end, :))/255, self.aVars.BEV.xg, self.aVars.BEV.yg, 'linear');
%                 imBEV = rot90(imBEV);
%                 imBEV = rot90(imBEV);
%                 
                %% Update the results display
                % Copy the current image to the results window
                figure(self.hResultsWindow);
                ax = gca(self.hResultsWindow);
                cla(ax)
%                 imshow(rgb, 'Parent', ax); hold on,
%                 imagesc(labels, 'Parent', ax); hold on,
                imagesc(imBEV, 'Parent', ax); colormap gray, hold on,
                title(sprintf('Time %0.2f', t))
               
                % Draw the VP Results
                self.aVars.VPTracker.PlotResults(ax, 0);
               
                %% Store the results window in a video
                if self.MakeVideo
                    im = getframe(ax);
                    writeVideo(mov, im.cdata);
                end                
                drawnow
            end
            if self.MakeVideo
                close(mov)
            end
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
    end
    
    methods(Static)
        function [X,Y] = ImageToWorld(v,u, params)

            m = params.m-1;
            n = params.n-1;
            alpha = params.alpha;
            dx = params.CameraLocationInWorld(1);
            dy = params.CameraLocationInWorld(2);
            dz = params.CameraLocationInWorld(3);
            theta = params.Theta;
            gamma = params.Gamma;

            X = dz * cot(theta - alpha + u.*(2*alpha/m)) .* sin(gamma - alpha + v.*(2*alpha / n)) + dx;
            Y = dz * cot(theta - alpha + u.*(2*alpha/m)) .* cos(gamma - alpha + v.*(2*alpha / n)) + dy;
        end
    end
end