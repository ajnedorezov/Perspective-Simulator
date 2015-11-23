function test_VPTracker_hough

    clc
    clear all
    close all

    % Highway driving, changing lanes
%     a =  mmreader('C:\Program Files (x86)\MATLAB\R2011a Student\toolbox\vision\visiondemos\viplanedeparture.avi');

%     vidFrames = read(a);
    a.NumberOfFrames = 101;
    quivlength = 30;
    onoff = 0;
    hFig1 = figure;
    ax1 = axes('Parent', hFig1);
    movegui(hFig1, 'northwest')

    % Initialize the video struct
    mov = struct([]);
    edges = struct([]);
    
    % Initialize the MC tracker
    numSamples = 1000;
    
    % Uniformly sample the size of the image
%     imsize = size(vidFrames(:,:,1,1));
    imsize = [960 720];
    prior.samples = rand(numSamples, 2)*diag(imsize(:));
    prior.mean = mean(prior.samples, 1);
    temp = bsxfun(@minus, prior.mean, prior.samples);
    prior.var = std(temp(:));
    
    % control = [0 0];    % For Highway video
    control = zeros(1, 2, a.NumberOfFrames);
%     control(:, 1, 50:83) = -4; % For city driving, car moving right so VP should move left.
    
    
    for k = 1:a.NumberOfFrames
    try
        fprintf('Processing frame %d of %d\n', k, a.NumberOfFrames)
        title(ax1, sprintf('Frame %d of %d', k, a.NumberOfFrames))
        drawnow

%         mov(k).cdata = vidFrames(:,:,:,k);
        fname = ['Test Images\CamSeq01\0016E5_0' num2str(2*(k-1) + 7959) '.png'];
        mov(k).cdata = imread(fname);
        mov(k).colormap = [];
        cla(ax1)
        imshow(mov(k).cdata, 'Parent', ax1)

        % Perform edge detection
        greyim = double(rgb2gray(mov(k).cdata));
        edges(k).cdata = edge(greyim, 'canny'); % More lines
%         edges(k).cdata = edge(greyim, 'sobel'); % Less lines
        edges(k).colomap = [];
        imedge = edges(k).cdata;
        imsize = size(imedge);
    
        % Find the image gradient
        xdir = imfilter(greyim, [-1 0 1]);
        ydir = imfilter(greyim, [-1 0 1]');
        magGradient = hypot(xdir, ydir);
        angGradient = atan2(xdir, ydir)+pi/2;
        
        % Wrap the angle of the gradient into [-pi,0]
        angGradient = wrapToPi(angGradient);
        angGradient(angGradient>0) = angGradient(angGradient>0) - pi;
        
        newxdir = cos(angGradient);
        newydir = sin(angGradient);
        
%        % Draw the direction of the x & y derivatives for each of the edges.
%         [xx, yy] = meshgrid(1:size(greyim,2), 1:size(greyim,1));
%         hold on
%         quiver(xx(imedge), yy(imedge), newxdir(imedge), newydir(imedge), 'color', 'b')

        % Compute the Hough Transfrom to create the line segments
        lines = computeHough(imedge);
            
        % Draw the Hough lines
        hold on
        numLines = length(lines);
        for l = 1:numLines
            x = lines(l).point1(1) + cos(lines(l).plottheta)*[-10000 0 10000];
            y = lines(l).point1(2) + sin(lines(l).plottheta)*[-10000 0 10000];
            
            plot(x, y, 'm', 'xliminclude', 'off', 'yliminclude', 'off')
        end
        
        % Create the A and b matrix
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
            
        
        % Compute the intercept points
        pts = findVert(A, b);
        if isempty(pts)
            disp('No intersections found')
            continue
        else
            plot(pts(1,:), pts(2,:), 'cx', 'markersize', 10, 'linewidth', 3)
            plot(mean(pts(1,:)), mean(pts(2,:)), 'mo', 'markersize', 10, 'linewidth', 2)
        end
        
        % Update the tracker
        temp = isnan(pts(1:2,:));
        pts(:, temp(1,:) | temp(2,:)) = [];
%         measurements = mean(pts(1:2,:), 2);
        measurements = pts(1:2,:)';
        posterior = monteCarloTracker_step(prior, control(:,:,k), measurements, imsize);       
        plot(posterior.samples(:,1), posterior.samples(:,2), 'y.')
        plot(posterior.mean(1), posterior.mean(2), 'gs', 'linewidth', 2);
        
        % The new posterior becomes the prior.
        prior = posterior;
        
    catch ME
        disp(ME.message)
        assignin('base', 'ME', ME)
        % One of the figures was closed stop break out of the loop
        delete(findall(0, 'Type', 'figure'))
        break
    end
    end

end

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