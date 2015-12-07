classdef VPTracker < handle
    properties
        numSamples
        imSize
        prior = struct('samples', [],...
                       'mean', [],...
                       'var', []);
        
        HoughLines = [];           
        InterceptPoints = [];
    end
    
    methods 
        function self = VPTracker(aNumSamples, aImSize)
            %% Initilize the Monte-Carlo Tracker(MCT)
            self.numSamples = aNumSamples;
            self.imSize = aImSize;
            
            % Uniformly sample the size of the image
            self.prior.samples = rand(aNumSamples, 2)*diag(aImSize);
            self.prior.mean = mean(self.prior.samples, 1);
            temp = bsxfun(@minus, self.prior.mean, self.prior.samples);
            self.prior.var = std(temp(:));
        end
        
        function Update(self, rgb, control)
            %% Perform edge detection
            greyim = double(rgb2gray(rgb));
            edges.cdata = edge(greyim, 'canny'); % More lines
    %         edges.cdata = edge(greyim, 'sobel'); % Less lines
            edges.colomap = [];
            imedge = edges.cdata;

            %% Compute the Hough Transfrom to create the line segments
            lines = self.computeHough(imedge);
            self.HoughLines = lines;

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
                return
            end

            if size(A, 1) < 2
                disp('Not enough lines to deterimine intersections')
                return
            end

            % Calculate the intercept points
            pts = self.findVert(A, b);
            if isempty(pts)
                disp('No intersections found')
                return
            end
            self.InterceptPoints = pts;

            %% Update the tracker
            temp = isnan(pts(1:2,:));
            pts(:, temp(1,:) | temp(2,:)) = [];
            measurements = pts(1:2,:)';
            posterior = self.monteCarloTracker_step(self.prior, control, measurements, self.imSize);       

            % The new posterior becomes the prior.
            self.prior = posterior;
        end
        
        function PlotResults(self, ax, detailLevel)
            if ~exist('detailLevel', 'var')
                detailLevel = 0;
            end
            
            if detailLevel >= 2
                % Draw the Hough lines
                for l = 1:length(self.HoughLines)
                    x = self.HoughLines(l).point1(1) + cos(self.HoughLines(l).plottheta)*[-10000 0 10000];
                    y = self.HoughLines(l).point1(2) + sin(self.HoughLines(l).plottheta)*[-10000 0 10000];

                    plot(ax, x, y, 'm', 'xliminclude', 'off', 'yliminclude', 'off')
                end

                % Display the intesections
                plot(ax, self.InterceptPoints(1,:), self.InterceptPoints(2,:), 'cx', 'markersize', 10, 'linewidth', 3)
%                     plot(ax, mean(self.InterceptPoints(1,:)), mean(self.InterceptPoints(2,:)), 'mo', 'markersize', 10, 'linewidth', 2)
            end
            
            % Display the vanishing point
            if detailLevel >= 1
                plot(ax, self.prior.samples(:,1), self.prior.samples(:,2), 'y.')
            end
            plot(ax, self.prior.mean(1), self.prior.mean(2), 'ys', 'linewidth', 2, 'Markersize', 10);
                               
        end
        
        function [x,y] = getVanishingPoint(self)
            x = self.prior.mean(1);
            y = self.prior.mean(2);
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