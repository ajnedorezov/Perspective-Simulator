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
            
            %% Initilize the Monte-Carlo Tracker(MCT)
            self.aVars.numSamples = 1000;
            
            % Uniformly sample the size of the image
            prior.samples = rand(self.aVars.numSamples, 2)*diag(self.aVars.imsize(:));
            prior.mean = mean(prior.samples, 1);
            temp = bsxfun(@minus, prior.mean, prior.samples);
            prior.var = std(temp(:));
            self.aVars.prior = prior;
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
            % 
            
            for t = 0:delT:80
                if ~ishandle(self.HD.MainView)
                    break
                end
                %% Update the vehicle/dynamics/scene simulation
                figure(self.hFigure);
                posvec = [t self.RoadParams.laneWidth*sin(t/25) + 3*self.RoadParams.laneWidth/2 self.CameraParams.height];
                targvec = posvec + self.DriverParams.lookAheadDistance*[1 self.RoadParams.laneWidth/25*cos(t/25) 0];
                campos(posvec);
                camtarget(targvec);
                
                control = [0 0];%[delT self.RoadParams.laneWidth*sin(t/25) + 3*self.RoadParams.laneWidth/2];

                %% Grab the image
                rgb = getframe(self.HD.MainView);
                rgb = rgb.cdata;
                
                %% Perform edge detection
                greyim = double(rgb2gray(rgb));
                edges.cdata = edge(greyim, 'canny'); % More lines
        %         edges.cdata = edge(greyim, 'sobel'); % Less lines
                edges.colomap = [];
                imedge = edges.cdata;
                
                %% Compute the Hough Transfrom to create the line segments
                lines = self.computeHough(imedge);
                
                %% Compute the intercept points
                % Create the A and b matrix
                numLines = length(lines);
                A = zeros(numLines, 2);
                b = zeros(numLines, 1);
                toDelete = [];
                for l = 1:numLines
                    A(l, :) = [-tan(lines(l).plottheta) 1];
                    b(l) = A(l, :)*lines(l).point1(:);
                    if isnan(A(l,1))
                        toDelete(end+1) = l;
                    end
                end
                A(toDelete, :) = [];
                b(toDelete) = [];
                if isempty(A)
                    % No lines were computed this frame.
                    continue
                end

                if size(A, 1) < 2
                    disp('Not enough lines to deterimine intersections')
                    continue
                end

                % Calculate the intercept points
                pts = self.findVert(A, b);
                if isempty(pts)
                    disp('No intersections found')
                    continue
                end
                
                %% Update the tracker
                temp = isnan(pts(1:2,:));
                pts(:, temp(1,:) | temp(2,:)) = [];
                measurements = pts(1:2,:)';
                posterior = self.monteCarloTracker_step(self.aVars.prior, control, measurements, self.aVars.imsize);       
                
                % The new posterior becomes the prior.
                self.aVars.prior = posterior;
                
                %% Update the results display
                % Copy the current image to the results window
                figure(self.hResultsWindow);
                ax = gca(self.hResultsWindow);
                cla(ax)
                imshow(rgb, 'Parent', ax); hold on,
                title(sprintf('Time %0.2f', t))
                
                % Draw the Hough lines
                for l = 1:numLines
                    x = lines(l).point1(1) + cos(lines(l).plottheta)*[-10000 0 10000];
                    y = lines(l).point1(2) + sin(lines(l).plottheta)*[-10000 0 10000];

                    plot(ax, x, y, 'm', 'xliminclude', 'off', 'yliminclude', 'off')
                end
                
                % Display the intesections
                plot(ax, pts(1,:), pts(2,:), 'cx', 'markersize', 10, 'linewidth', 3)
%                 plot(ax, mean(pts(1,:)), mean(pts(2,:)), 'mo', 'markersize', 10, 'linewidth', 2)
                
                % Display the vanishing point
                plot(ax, posterior.samples(:,1), posterior.samples(:,2), 'y.')
                plot(ax, posterior.mean(1), posterior.mean(2), 'gs', 'linewidth', 2);
                               
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
                  
            h.trunk = surface(trunkx + x, trunky + y, trunkz, 'FaceColor', [92 64 51]/255);
            h.top = surface(topx + x, topy + y, topz, 'FaceColor', [25 50 25]/255);
        end
    end
    
    %% Methods for the MCT
    methods(Static)
        function lines = computeHough(edges)
    
    % Perform the Hough transform
    [H,T,R] = hough(edges);
    
    % Find the peaks in the Hough transform accumulator matrix
    % corresponding to the line estimates
    P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));

    % Find the equations fo the lines
    lines = houghlines(edges,T,R,P,'FillGap',.25*min(size(edges)),'MinLength',7);
    for k = 1:length(lines)
        xy = [lines(k).point1; lines(k).point2];
        xydiff = diff(xy, 1);
        lines(k).plottheta = atan2(xydiff(2), xydiff(1));
    end

end

        function pts = findVert(A, b)
            pts = [];

            [numConstraints, numVariables] = size(A);

            % If num constraints > num variables
            if numConstraints > numVariables
                A = [A eye(numConstraints)];
                [numConstraints, numVariables] = size(A);
            end

            ind = nchoosek(1:numVariables, numConstraints);
            % Ignore combinations that don't use the first two variables
            temp = (ind == 1 | ind == 2);
            ind(~(temp(:,1) & temp(:,2)), :) = [];

            numCombinations = size(ind, 1);%nchoosek(n,m);
            for k = 1:numCombinations
                y = zeros(numVariables,1);
                % Solve the eqution using the current set of variables
                x = A(:,ind(k,:))\b;
                if all(isinf(x) | isnan(x))
                    % Ingore parallel lines
                    continue
                end

                % Store the values into the proper indicies
                y(ind(k,:)) = x;
                pts = [pts y];
            end
        end

        function posterior = monteCarloTracker_step(prior, control, measurement, imsize)
            pixelVar = 2; % # pixels

            [numSamples numDimensions] = size(prior.samples);
            trueNumSamples = numSamples;

            % Apply the control to each sample 
            prediction = bsxfun(@plus, prior.samples, control);

            % Compare the observation to our prediction. This is how different the
            % measurement and samples are.
            numMeas = size(measurement, 1);
            Innovation = nan(numSamples, numDimensions, numMeas);
            for m = 1:numMeas
                Innovation(:,:,m) = bsxfun(@minus, measurement(m,:), prediction);
            end

            % Compute the new liklihood using gaussian distribution
            % exp(-1/2*(x-mu)'*sig^-1*(x-mu))
            L = ones(numSamples, numMeas) / numSamples;
            varInv = 1/prior.var^2*eye(numDimensions);
            for m = 1:numMeas
                for s = 1:numSamples
                    L(s,m) = exp(-0.5*Innovation(s,:,m)*varInv*Innovation(s,:,m)');
                end
            end
            L = sum(L, 2);

            % Update the distribution
            CDF = cumsum(L)/sum(L);
            if any(diff(CDF) == 0)
                inds = diff(CDF) == 0;
                CDF(inds) = [];
                prediction(inds,:) = [];
                numSamples = size(prediction, 1);
            end

            if any(L)
                % Draw the samples from prediction distribution based on the
                % weights
                newSamplesIndex = interp1(CDF, 1:numSamples, rand(trueNumSamples, 1), 'nearest', 'extrap');
                posterior.samples = prediction(newSamplesIndex, :) + pixelVar*randn(trueNumSamples, numDimensions);
                posterior.sampleIndex = newSamplesIndex;
                posterior.mean = mean(posterior.samples, 1);
                temp = bsxfun(@minus, posterior.mean, posterior.samples);
                posterior.var = std(temp(:));
            else
                % The weight for all samples is zero. I.E. the innovation is too
                % small to be usefull.
                % Resample using the current mean to create variations.
                disp('RESAMPLING')
                % Have to resample the distribution
                posterior.mean = mean(prediction, 1);
        %         posterior.var = .0125;
        %         posterior.var = .00125;
                posterior.var = 2*pixelVar;
                posterior.samples = bsxfun(@plus, randn(numSamples, numDimensions)*posterior.var, posterior.mean);        
            end

        end

        function y = wrapToPi(x)
            % Makes x between [-pi pi]
            y = mod(x, sign(x)*2*pi);
            y(abs(y)>pi) = y(abs(y)>pi) - sign(y(abs(y)>pi))*2*pi;
        end
    end
    
end