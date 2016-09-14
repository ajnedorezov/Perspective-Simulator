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
        
        MakeVideo = true;
%         MakeVideo = false;
        
        Lighting
    end
    
    properties(Constant)
        mi2m = 1609.34;     % Miles to Meters
        
        SimulationDistance = 1 * 1609.34; % 1 mile
        SimulationParams = struct('numCars', 2);

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
%             carlane = [2 0 1 1];
            carlane = [1 2];
%             carlane = [0];
            for n = 1:self.SimulationParams.numCars
%                 self.MovingObjects.Cars(n) = surface(x + 10 + 25*n, y + (carlane(n)+0.5)*self.RoadParams.laneWidth - self.CarParams.width/2, z, 'FaceColor', self.CarParams.color, 'UserData', self.CarParams.color);
                self.MovingObjects.Cars(n) = surface(x + 10 + 25, y + (carlane(n)+0.5)*self.RoadParams.laneWidth - self.CarParams.width/2, z, 'FaceColor', self.CarParams.color, 'UserData', self.CarParams.color);
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
                mov = VideoWriter('Examples\CamSeqManipulation\PathIntersectingObstacles-Case4.avi');
                mov.FrameRate = round(1/delT);
                open(mov);
            end
            
            % Create the initial snake coordinates
%             imsize = [545 1201];
            imsize = [1201 545];
%             x_s = linspace(imsize(1)/2,imsize(1)/2,10);
%             y_s = linspace(80,imsize(2),10);
            snakeX = linspace(0,imsize(1),10);
            snakeY = linspace(imsize(2)/2,imsize(2)/2,10);

            % Upsample & create a spline
            steps = 0:(length(snakeX)-1);
            newSteps = 0:0.05:(length(snakeX)-1);
            pp = spline(steps,[snakeX' snakeY']', newSteps);
            snakeX = pp(1,:)';
            snakeY = pp(2,:)';
            
            % For a car traveling at 20m/s (~45mph) it will take ~80sec to
            % travel 1 mile
            tic
%             posvec = [-29*delT 5*self.RoadParams.laneWidth/2 self.CameraParams.height];
            posvec = [10 5*self.RoadParams.laneWidth/2 self.CameraParams.height];
            commandedHeading = 0;
            currentYaw = 0;
            commandedSpeed = 29; % Travel @ 65 mph
            currentSpeed = commandedSpeed;
            intersectionCounter = 1;
            
            endTime = 7.5;
            downRangeToObstacleOnPath = inf(1, length(0:delT:endTime));
            closestDownRangeToObstacle = inf(1, length(0:delT:endTime));
            for t = 0:delT:endTime% 34.25;%
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
                
                %% Update the VP Tracker
                control = [0 0];%[delT self.RoadParams.laneWidth*sin(t/25) + 3*self.RoadParams.laneWidth/2];
                self.aVars.VPTracker.Update(rgb, control);
                
                %% Do color thresholding to locate and classify the road/obstacles
                binaryIm = zeros(size(rgb,1), size(rgb,2), 3);
                for n = 1:3
                    channel = rgb(:,:,n);
                    roadRegion = channel(self.aVars.sampleRegionI, self.aVars.sampleRegionJ);
                    avgPixelInt = mean(roadRegion(:));
                    binaryIm(:,:,n) = channel < (avgPixelInt-self.aVars.intensityRange) | channel > (avgPixelInt + self.aVars.intensityRange);
                end
                ind = sum(binaryIm,3)==0;
                
                %% Perform the IPM
                % Transform the image
                for n = 1:3
                    channel = binaryIm(:,:,n);
                    channel(ind) = 0;
                    ipmIm(:,:,n) = self.aVars.IPM.performTransformation(double(channel));
                    
                    channel = rgb(:,:,n);
                    rgbIPM(:,:,n) = self.aVars.IPM.performTransformation(double(channel));
                end
                
                % Transform the vanishing point
                ipmVP = self.aVars.IPM.transformSinglePoint(max(self.aVars.VPTracker.prior.mean(2), self.aVars.IPM.rHorizon), self.aVars.VPTracker.prior.mean(1));
                
                % Get the current vanishing point estimate
                origvpx = (ipmVP(1)-self.aVars.IPM.xRange(1))*size(ipmIm,2)/diff(self.aVars.IPM.xRange);
                origvpy = (ipmVP(2)-self.aVars.IPM.yRange(1))*size(ipmIm,1)/diff(self.aVars.IPM.yRange);
                                
                % Limit vp to point in image
                m = (origvpx-size(ipmIm,2)/2)/origvpy;
                vpy = size(ipmIm,1);
                vpx = vpy*m + size(ipmIm,2)/2;

                
                %% Detect obstacles by checking if its a vertical streak
%                 useKmeans = true;
                useKmeans = false;

                if useKmeans
                    numCusters = 3;
                    grayIm = rgb2gray(ipmIm);
            %         grayIm = double(rgb2gray(uint8(rgbIPM)));

                    grayIm(isnan(grayIm)) = -1;
                    [IDX, centers] = kmeans(grayIm(:), numCusters);
                    IDX = reshape(IDX, size(grayIm));
                    IDX(IDX == IDX(1,1)) = -1;     % Grab the first pixel since this is always black because the IPM creates a cone and this pixel is outside of the interpolated data

                    % Figure out the color of the roadway
            %         roadPixel = grayIm(round(size(grayIm,2)/2) + (-5:5), 80+(0:5));
                    roadPixel = grayIm(ptInFrontOfCar(1) + (-20:20), ptInFrontOfCar(2)+(-10:30)); %ptInFrontOfCar = [400 330];
                    roadPixelRange = median(roadPixel(:)) + 50*[-1 1];%0.05*[-1 1];

                    % Break each cluster into individual unique clusters,
                    % concurrently, decide if the cluster is roadway
                    newLabels = zeros(size(IDX));
                    offset = 0;
                    roadLabels = [];
                    for n = 1:length(centers)
                        ind = IDX == n;
                        tempLabels = bwlabeln(ind) + offset;
                        newLabels(ind) = tempLabels(ind & newLabels==0);
                        offset = max(newLabels(:));
                        % NOTE this conditional logic needs to change so it'll be more adaptive
                        % to roadway light fluctuations (i.e. use the color of the road right
                        % in front of the camera, this means we need to maintain some distance
                        % between vehicles)
                    %     if centers(n) > .33 && centers(n) < .4
                        if centers(n) > roadPixelRange(1) && centers(n) < roadPixelRange(2)
                            % These belong to the road
                            roadLabels = [roadLabels; unique(tempLabels)];
                        end
                    end
                else
                    grayIm = sum(ipmIm,3) > 0;
                    newLabels = bwlabeln(grayIm);
                end

                % Get the region properties for the segments
                stats = regionprops(newLabels, 'BoundingBox', 'Extent', 'Orientation');

                % Decide if the cluster is streak like and something that
                % should be avoided
                obstacles = false(length(stats),1);
                for n = 1:length(stats)
                    obstacles(n) = stats(n).BoundingBox(4) > 100 && stats(n).BoundingBox(4) > stats(n).BoundingBox(3) && stats(n).BoundingBox(3) > 30;
                end
%                 isObstacle(:,:,1) = ismember(newLabels, find(obstacles));
                isObstacle(:,:,1) = imdilate(ismember(newLabels, find(obstacles)), true(1,35));
                
                %% Do the maze path following algorithm stuff

                % Smooth the image and convert to an edge map
                smArea = 50;
                se = strel('ball', smArea, smArea);
                GI = imdilate(double(isObstacle), se)-smArea;
                f = smArea-GI;
                
                % Compute the GVF of the edge map f
                [px,py] = GVF(f, 0.2, 40);
                
%                 % Make the magnitude of all vectors equal
%                 magGVF = hypot(px,py) + 1e-10;
%                 px = px./magGVF;
%                 py = py./magGVF;

                % Make the obstacle have vectors towards the vanishing point
                imsize = size(px);
                
                % Apply a clockwise/counterclockwise rotation around the edges
                LeftRight = [zeros(size(f,1),1) diff(f, [], 2)];
                UpDown = [zeros(1,size(f,2)); diff(f, [], 1)];
                maxEdge = 1;

                clockwise = true;
%                 clockwise = false;
                if clockwise 
                    py(LeftRight < 0) = -maxEdge/2;    px(LeftRight < 0) = 0;
                    py(LeftRight > 0) = maxEdge/2;     px(LeftRight > 0) = 0;
                    py(UpDown < 0) = 0;
                    py(UpDown > 0) = 0;
                    px(UpDown < 0) = maxEdge/2;
                    px(UpDown > 0) = -maxEdge/2;
                else
                    py(LeftRight < 0) = maxEdge/2;    px(LeftRight < 0) = 0;
                    py(LeftRight > 0) = -maxEdge/2;     px(LeftRight > 0) = 0;
                    px(UpDown < 0) = -maxEdge/2;
                    px(UpDown > 0) = maxEdge/2;
                end    
                
                % Apply a slope towards the goal point
                towardsGoal = true;
%                 towardsGoal = false;
                [cc,rr] = meshgrid(1:imsize(2), 1:imsize(1));
                if towardsGoal
                    dx = vpx - cc;
                    dy = vpy - rr;
                else
                    dx = vpx - cc;
                    dy = vpy - rr;
                end
                newMag =  sqrt(dx.*dx + dy.*dy) + eps;
                newMag = 1 - newMag./max(max(newMag));
                [fx2,fy2] = gradient(newMag);
                mag = hypot(fx2,fy2) + eps;
                fx2 = fx2./mag;
                fy2 = fy2./mag;

                pmag = max(max(hypot(px,py)));
                px = px/pmag;
                py = py/pmag;

                px(isObstacle) = fx2(isObstacle);
                py(isObstacle) = fy2(isObstacle);

                % Make the magnitude of all vectors equal
                magGVF = hypot(px,py) + 1e-10;
                px = px./magGVF;
                py = py./magGVF;

%                 [cc,rr] = meshgrid(1:imsize(2), 1:imsize(1));
%                 dy = vpy - rr;
%                 dx = vpx - cc;
%                 newMag =  sqrt(dx.*dx + dy.*dy) + eps;
% 
%                 px(isObstacle)  = 0.75*px(isObstacle) + 0.25*dx(isObstacle)./newMag(isObstacle);
%                 py(isObstacle)  = 0.75*py(isObstacle) + 0.25*dy(isObstacle)./newMag(isObstacle);

%                 % Plot the gradient vectors
%                 [qx,qy] = meshgrid(1:10:imsize(1), 1:10:imsize(2));
%                 ind = sub2ind(imsize, qx,qy);
%                 subplot(122), quiver(qy,qx,px(ind),py(ind)); set(gca, 'ydir', 'normal','xdir','reverse')

                % Initialize the snake
                if t == 0 
                    snakeTime = linspace(0,1, 100)';
                    cx = floor(imsize(2)/2);
                    cy = 1;
                    snakeX = cx + snakeTime.*(vpx-cx); % vpx.*t + (1-t).*cx;
                    snakeY = cy + snakeTime.*(vpy-cy);
                else
                    snakeX(end) = vpx;
                    snakeY(end) = vpy;
                end
% 
%                 [snakeX, snakeY] = snakedeform(x,y,1,0.75,0.5,25,px,py,5*5);
%                 [snakeX,snakeY] = snakedeform(x,y,1,0.75,0.25,25,px,py,5*20);
%                 [snakeX,snakeY] = snakedeform(snakeX,snakeY,1,0.75,0.25,20,px,py,25); %*
                [snakeX,snakeY] = snakedeform(snakeX,snakeY,1,0.25,0.25,5,px,py,25);
                
                %% Convert the snake into a commanded heading
                xcenter = size(ipmIm,2)/2;
                sx = diff(self.aVars.IPM.xRange)/size(ipmIm,2);
                sy = diff(self.aVars.IPM.yRange)/size(ipmIm,1);
                
                % Grab a point ~50ft in down the curve (i.e.
                % 50ft/300ft*100pts --> 17th index)
                drInd = interp1(snakeY*sy, 1:length(snakeY), 50, 'nearest');
                if isempty(drInd) || isnan(drInd)
                    % rough estimate: 50ft/300ft*100pts --> 17th index
                    drInd = 17;
                end
                ptX = (xcenter - snakeX(drInd))*sx;
                ptY = snakeY(drInd)*sy;
                commandedHeading = atan2(ptX,ptY);

                %% Update the results display
                
%                 % Draw the scene
%                 figure(999)
%                 ax = gca(figure(999));
%                 cla(ax);
%                 imshow(rgb); hold on
%                 
%                 % Draw the VP Results
%                 self.aVars.VPTracker.PlotResults(ax, 0);
%                 self.aVars.VPTracker.PlotResults(ax, 2);
                
                % Copy the current image to the results window
                figure(self.hResultsWindow);
                ax = gca(self.hResultsWindow);
                cla(ax)
                
                tIm = imoverlay(uint8(rgbIPM), uint8(isObstacle), [1 0 0]);
                imshow(rot90(tIm,2), 'Parent', ax, 'XData', (self.aVars.IPM.xRange), 'YData', fliplr(self.aVars.IPM.yRange)), hold(ax, 'on')
                title(sprintf('Time %0.2f', t))
                axis equal tight on
                set(ax,'yDir','normal')%,'xdir','reverse')
                ylabel(ax, 'Down Range (ft)')
                xlabel(ax, 'Cross Range (ft)')
                
                % Draw bounding boxes around the obstacles
%                 for n = find(obstacles)'
%                     plot(ax, stats(n).BoundingBox(1) + [0 0 stats(n).BoundingBox([3 3]) 0], stats(n).BoundingBox(2) + [0 stats(n).BoundingBox([4 4]) 0 0], 'y', 'linewidth', 1.5);
%                 end

                % Draw the path to the vanishing point
%                 plot(ax, [imsize(2)/2 vpx], [0 vpy], 'g--', 'linewidth', 2)
%                 plot(imsize(2)/2, 0, 'x', vpx, vpy, 'o')
                
                % Draw the path planning results
                plot(ax, (xcenter-snakeX)*sx, snakeY*sy, 'Color', [0.5 1 0], 'linewidth', 2); 
                plot(ax, 0, 0, 'x', (vpx-xcenter)*sx, vpy*sy, 'o')
                quiver(0,0, ptX, ptY, 'c', 'Parent', ax, 'LineWidth', 3)
                
%                 keyboard

                %%
                obstacleOnPath = interp2(double(isObstacle), snakeX, snakeY, 'nearest');
                obstacleIntersectionIndex = find(obstacleOnPath==1, 1, 'first');
                if ~isempty(obstacleIntersectionIndex)
                    downRangeToObstacleOnPath(intersectionCounter) = snakeY(obstacleIntersectionIndex)*sy;
                    
                    plot(ax, (xcenter-snakeX(obstacleIntersectionIndex))*sx, downRangeToObstacleOnPath(intersectionCounter), 'cx', 'markersize', 10, 'linewidth', 3);
                else
                    downRangeToObstacleOnPath(intersectionCounter) = inf;
                end
                
                closestObstacle = isObstacle(:, 272);
                closestObstacleIntersectionIndex = find(closestObstacle==1, 1, 'first');
                if ~isempty(closestObstacleIntersectionIndex)
                    closestDownRangeToObstacle(intersectionCounter) = closestObstacleIntersectionIndex*sy;
                end
                
                intersectionCounter = intersectionCounter + 1;
                
                %% Store the results window in a video
                if self.MakeVideo
                    im = getframe(ax);
                    writeVideo(mov, im.cdata);
                end                
                drawnow
            end
            
%             dataToSave.downRangeToObstacleOnPath = downRangeToObstacleOnPath;
%             dataToSave.closestDownRangeToObstacle = closestDownRangeToObstacle;
%             save('Examples\CamSeqManipulation\downRangeToObstacle - Case 2.mat', '-struct', 'dataToSave')
            figure, plot(1:(intersectionCounter-1), downRangeToObstacleOnPath, 'bo', 1:(intersectionCounter-1), closestDownRangeToObstacle, 'ro')
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