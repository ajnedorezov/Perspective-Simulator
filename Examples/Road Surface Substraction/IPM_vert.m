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
            
%             tempImSize = [self.ImSize(1)-self.rHorizon+1, self.ImSize(2)];
%             indIm = reshape(1:prod(tempImSize), tempImSize);
            
%             self.Indices = griddata(self.XWorld, self.YWorld, indIm, self.xSteps, self.ySteps, 'nearest');
            %% Extract the weights and indices to compute the bilinear interpolation
            ox = self.xSteps;
            oy = self.ySteps;
            nx = self.XWorld;
            ny = self.YWorld;
            
            x1 = ones(size(nx));
            x2 = ones(size(nx));
            y1 = ones(size(ny));
            y2 = ones(size(ny));
            
            
            for r = 1:size(nx,1)
                for c = 1:size(nx,2)
                    b = ox - nx(r,c);
                    b1 = b;
                    b2 = b;
                    b1(b>0) = nan;
                    b2(b<=0) = nan;

                    [~,x1(r,c)] = min(abs(b1), [], 2);
                    [~,x2(r,c)] = min(abs(b2), [], 2);

                    b = oy - ny(r,c);
                    b1 = b;
                    b2 = b;
                    b1(b>0) = nan;
                    b2(b<=0) = nan;

                    [~,y1(r,c)] = min(abs(b1), [], 2);
                    [~,y2(r,c)] = min(abs(b2), [], 2);
                end
            end
            
            % We have the indicies, now let's find the weights
            ind00 = sub2ind(size(x), y1, x1);
            ind01 = sub2ind(size(x), y1, x2);
            ind10 = sub2ind(size(y), y2, x1);
            ind11 = sub2ind(size(y), y2, x2);

            denom = (x2-x1).*(y2-y1);
            b11 = (x2-nx).*(y2-ny)./denom;
            b12 = (x2-nx).*(ny-y1)./denom;
            b21 = (nx-x1).*(y2-ny)./denom;
            b22 = (nx-x1).*(ny-y1)./denom;

            
            self.Indices = {ind00 ind01; ind10 ind11};
            self.Weights = {b11 b12; b21 b22};
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

            outIm = Icropped(self.Indices);

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