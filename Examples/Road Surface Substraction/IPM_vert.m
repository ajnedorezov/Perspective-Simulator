classdef IPM_vert < handle
    properties
        ImSize = [1 1]
        Static = true;
        
        % IPM Parameters
        alpha_tot = 20*pi/180;
        
        cameraX = 0;
        cameraY = 0;
        cameraZ = 0;
        gamma = 0;
        theta = pi/180;
        
        XWorld
        YWorld
        
        stepSize = 0.125;
        xRange = [-34 34];
        yRange = [10 200];
        
        alpha
        rHorizon
        xSteps
        ySteps
        
        Indices
        Weights
    end
    
    properties%(Dependent = true)
        reEvalParams = true;
    end
    
    methods
        function self = IPM_vert(varargin)
            
            ip = inputParser;
            ip.addRequired('ImSize');
            ip.addOptional('Static', self.Static);
            ip.addOptional('alpha_tot', self.alpha_tot);
            ip.addOptional('cameraX', self.cameraX);
            ip.addOptional('cameraY', self.cameraY);
            ip.addOptional('cameraZ', self.cameraZ);
            ip.addOptional('gamma', self.gamma);
            ip.addOptional('theta', self.theta);
            ip.addOptional('stepSize', self.stepSize, @(x) all(x > 0));
            ip.addOptional('xRange', self.xRange);
            ip.addOptional('yRange', self.yRange);
            
            ip.parse(varargin{:});
            
            fnames = fieldnames(ip.Results);
            for n = 1:length(fnames)
                self.(fnames{n}) = ip.Results.(fnames{n});
            end        
                        
            if self.Static
                % Precompute transformation from Camera to World Frame
                self.computeTransform();
            end
        end
        
        function set.ImSize(self, val)
            self.ImSize = val;
            self.reEvalParams = true;
        end
        
        function set.alpha_tot(self, val)
            self.alpha_tot = val;
            self.reEvalParams = true;
        end
        
        function set.theta(self, val)
            self.theta = val;
            self.reEvalParams = true;
        end
        
        function set.xRange(self, val)
            self.xRange = val;
            self.reEvalParams = true;
        end
        
        function set.yRange(self, val)
            self.yRange = val;
            self.reEvalParams = true;
        end        
        
        function computeTransform(self)
            
            if self.reEvalParams
                den = norm(self.ImSize-1);
                self.alpha = atan( (self.ImSize(1)-1)/den * tan(self.alpha_tot) );

                self.rHorizon = ceil( (self.ImSize(1)-1) / (2*self.alpha) * (self.alpha - self.theta) + 1) + 10;

                if length(self.stepSize) > 1
                    xStepSize = self.stepSize(1);
                    yStepSize = self.stepSize(2);
                else
                    xStepSize = self.stepSize;
                    yStepSize = self.stepSize;
                end
                self.xSteps = max(self.xRange):-xStepSize:min(self.xRange);

                self.ySteps = (min(self.yRange):yStepSize:max(self.yRange))';

                self.reEvalParams = false;
            end
            
            m = self.ImSize(1);
            n = self.ImSize(2);
            
            mCropped = m-self.rHorizon+1; % rows in the cropped image.

            Xvis = zeros(mCropped,n);
            Yvis = zeros(size(Xvis));

            for r = 1:mCropped

                rOrig = r + self.rHorizon - 1;
                u = rOrig - 1;

                for c = 1:n
                    v = c - 1;
                    cotStuff = cot(self.theta-self.alpha+u*(2*self.alpha)/(m-1));
                    otherArg = self.gamma-self.alpha+v*(2*self.alpha)/(n-1);
                    Xvis(r,c) = self.cameraZ*cotStuff*sin(otherArg) + self.cameraX;
                    Yvis(r,c) = self.cameraZ*cotStuff*cos(otherArg) + self.cameraY;
                end
            end
            
            self.XWorld = Xvis;
            self.YWorld = Yvis;
            
            tempImSize = [self.ImSize(1)-self.rHorizon+1, self.ImSize(2)];
            indIm = reshape(1:prod(tempImSize), tempImSize);
            
%             self.Indices = griddata(self.XWorld, self.YWorld, indIm, self.xSteps, self.ySteps, 'nearest');

            %% Extract the weights and indices to compute the bilinear interpolation
            ind1 = ones(length(self.ySteps), length(self.xSteps));
            ind2 = ind1;
            ind3 = ind1;
            b1 = zeros(length(self.ySteps), length(self.xSteps));
            b2 = b1;
            b3 = b1;
            tic

            TRI = delaunay(self.XWorld,self.YWorld);
            for r = 1:length(self.ySteps)
                for c = 1:length(self.xSteps)
                    % Find the triangles touching the closest point
                    d = (self.XWorld(:) - self.xSteps(c)).^2 + (self.YWorld(:) - self.ySteps(r)).^2;
                    [~, ind] = min(d);

                    triInd = find(any(ismember(TRI, ind), 2));

                    for t = triInd'
                        % Loop over the different triangles and find the one that
                        % contains this point

                        tx = self.XWorld(TRI(t,:));
                        ty = self.YWorld(TRI(t,:));

                        A = [ones(1,3);
                             tx
                             ty];

                        Ai = A; Ai(2:3,1) = [self.xSteps(c);self.ySteps(r)];
                        Aj = A; Aj(2:3,2) = [self.xSteps(c);self.ySteps(r)];
                        Ak = A; Ak(2:3,3) = [self.xSteps(c);self.ySteps(r)];
                        detA = det(A);
                        w1 = det(Ai)/detA;
                        w2 = det(Aj)/detA;
                        w3 = det(Ak)/detA;

                        if all(sign([w1 w2 w3]) >= 0) || all(sign([w1 w2 w3]) <= 0)
                            ind1(r,c) = TRI(t,1);
                            ind2(r,c) = TRI(t,2);
                            ind3(r,c) = TRI(t,3);
                            b1(r,c) = w1;
                            b2(r,c) = w2;
                            b3(r,c) = w3;
                            break
                        end
                    end 
                end

                disp(['Row: ' num2str(r) ' of ' num2str(length(self.ySteps))])

            end
            
            self.Indices = {ind1 ind2 ind3};
            self.Weights = {b1 b2 b3};
        end
        
        function ptWorld = transformSinglePoint(self, x, y)
            if ~exist('y','var')
               if length(x) == 2
                   y = x(2);
                   x(2) = [];
               else
                   error('Invalid Y value');
               end
            end 
            
            if self.reEvalParams
                den = norm(self.ImSize-1);
                self.alpha = atan( (self.ImSize(1)-1)/den * tan(self.alpha_tot) );

                self.rHorizon = ceil( (self.ImSize(1)-1) / (2*self.alpha) * (self.alpha - self.theta) + 1) + 10;

                self.xSteps = max(self.xRange):-self.stepSize:min(self.xRange);

                self.ySteps = (min(self.yRange):self.stepSize:max(self.yRange))';

                self.reEvalParams = false;
            end
            
            cotStuff = cot(self.theta-self.alpha+(x(:)'-1)*(2*self.alpha)/(self.ImSize(1)-1));
            otherArg = self.gamma-self.alpha+(y(:)'-1)*(2*self.alpha)/(self.ImSize(2)-1);
            ptWorld(1,:) = self.cameraZ.*cotStuff.*sin(otherArg) + self.cameraX;
            ptWorld(2,:) = self.cameraZ.*cotStuff.*cos(otherArg) + self.cameraY;
            
        end
        
        function outIm = performTransformation(self, grayIm, showFig)
            if ~exist('showFig', 'var')
                showFig = -1;
            end
            Icropped = grayIm(self.rHorizon:end,:);
            
%             outIm = griddata(self.XWorld, self.YWorld, Icropped, self.xSteps, self.ySteps, 'linear');
%             outIm = griddata(self.XWorld, self.YWorld, Icropped, self.xSteps, self.ySteps, 'nearest');
%             F = scatteredInterpolant(self.XWorld(:), self.YWorld(:), Icropped(:));
%             [xx,yy] = meshgrid(self.xSteps, self.ySteps);
%             outIm = F(xx,yy);

%             outIm = Icropped(self.Indices);
%             outIm = zeros([length(self.ySteps) length(self.xSteps),3]);
%             for n = 1:3
%                 outIm(:,:,n) = im(ind00).*b11 ...
                outIm = Icropped(self.Indices{1}).*self.Weights{1} ...
                        + Icropped(self.Indices{2}).*self.Weights{2} ...
                        + Icropped(self.Indices{3}).*self.Weights{3};
%             end

            if showFig > 0
                figure(showFig)
                                
                surf(self.YWorld, -self.XWorld, zeros(size(self.XWorld)), 'FaceColor', 'interp', ...
                    'CData', floor(Icropped*255), 'EdgeColor', 'none');
                % surf(Yvis, -Xvis, zeros(size(Xvis)), 'FaceColor', 'interp', ...
                %     'CData', floor(rgb2gray(Icropped)*255), 'EdgeColor', 'none');
                view(2);
                axis image;
                colormap(gray);
                set(gca, 'YLim', [-40 40]);
                title('Bertozzi and Broggi''s Inverspse Perspective Mapping Results', 'FontSize', 10);
                xlabel('Forward Distance (feet)');
                ylabel('Side to Side Distance (feet)');

                % For comparison, also plot where the lane markers and stop line should be
                % according to the way they were generated.  (Remember that this method is
                % supposed to take care of the offset between the camera and the origin.)
                width = 1.5;
                hOut = line([self.YWorld(end,1), self.YWorld(1,1)], [-9.84252, -9.84252], [0 0], 'LineStyle', '--', ...
                    'Color', 'r', 'LineWidth', width);
                line([self.YWorld(end,1), self.YWorld(1,1)], [9.84252, 9.84252], [0 0], 'LineStyle', '--', ...
                    'Color', 'r', 'LineWidth', width);
                hCent = line([self.YWorld(end,1), self.YWorld(1,1)], [0, 0], [0 0], 'LineStyle', '--', ...
                    'Color', 'b', 'LineWidth', width);
                hStop = line([76, 76], [-9.84252, 0], [0 0], 'LineStyle', '--', ...
                    'Color', 'y', 'LineWidth', width);
                legend([hOut, hCent, hStop], {'Ideal Outside', 'Ideal Center', 'Ideal Stop Line'}, ...
                    'Location', 'NEO');

                % The lane markers and stop line in the mapped image don't come out that
                % close using this method.  The distance to the lane marker is particularly
                % erroneous.
            end
        end
    end
end