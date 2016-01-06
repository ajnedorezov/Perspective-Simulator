classdef IPM < handle
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
    end
    
    properties%(Dependent = true)
        reEvalParams = true;
    end
    
    methods
        function self = IPM(varargin)
            
            ip = inputParser;
            ip.addRequired('ImSize');
            ip.addOptional('Static', self.Static);
            ip.addOptional('alpha_tot', self.alpha_tot);
            ip.addOptional('cameraX', self.cameraX);
            ip.addOptional('cameraY', self.cameraY);
            ip.addOptional('cameraZ', self.cameraZ);
            ip.addOptional('gamma', self.gamma);
            ip.addOptional('theta', self.theta);
            ip.addOptional('stepSize', self.stepSize, @(x) x > 0);
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

                self.xSteps = max(self.xRange):-self.stepSize:min(self.xRange);

                self.ySteps = (min(self.yRange):self.stepSize:max(self.yRange))';

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
        end
        
        function outIm = performTransformation(self, grayIm)
            Icropped = grayIm(self.rHorizon:end,:);
            
            outIm = griddata(self.XWorld, self.YWorld, Icropped, self.xSteps, self.ySteps, 'linear');
%             outIm = griddata(self.XWorld, self.YWorld, Icropped, self.xSteps, self.ySteps, 'nearest');
%             F = scatteredInterpolant(self.XWorld(:), self.YWorld(:), Icropped(:));
%             [xx,yy] = meshgrid(self.xSteps, self.ySteps);
%             outIm = F(xx,yy);
        end
    end
end