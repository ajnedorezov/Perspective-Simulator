function test_VPTracker_orig_try2

    clc
    clear all
    close all

    % Highway driving, changing lanes
%     a =  mmreader('C:\Program Files (x86)\MATLAB\R2011a Student\toolbox\vision\visiondemos\viplanedeparture.avi');

%     vidFrames = read(a);
%     vidFrames = a.frame;
    a.NumberOfFrames = 101;
    quivlength = 30;
    onoff = 0;
    hFig1 = figure;
    ax1 = axes('Parent', hFig1);
%     hFig2 = figure;
%     ax2 = axes('Parent', hFig2);
    movegui(hFig1, 'northwest')
%     movegui(hFig2, 'southwest')

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
    control = [0 0];
    
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
    
        % Find the image gradient
        xdir = imfilter(greyim, [-1 0 1]);
        ydir = imfilter(greyim, [-1 0 1]');
%         magGradient = hypot(xdir, ydir);
        angGradient = atan2(xdir, ydir)+pi/2;
        
        % Wrap the angle of the gradient into [-pi,0]
        angGradient = wrapToPi(angGradient);
        angGradient(angGradient>0) = angGradient(angGradient>0) - pi;
        
        newxdir = cos(angGradient);
        newydir = sin(angGradient);
        
       % Draw the direction of the x & y derivatives for each of the edges.
        [xx, yy] = meshgrid(1:size(greyim,2), 1:size(greyim,1));
%         figure(1)
        hold on
        quiver(xx(imedge), yy(imedge), newxdir(imedge), newydir(imedge), 'color', 'b')

        % Break the image into a much coarser grid and average the x & y
        % derivates within that grid space;
        gridsize = 3;
        [sx, sy] = size(greyim);
        xstartind = round(linspace(1, sy, gridsize));
        ystartind = round(linspace(1, sx, gridsize));

        % Compute the average gradient for each grid space
        sector = struct;
        numbins = 5;
        for xind = 1:length(xstartind)-1
            for yind = 1:length(ystartind)-1
                edgeindex = imedge(ystartind(xind):ystartind(xind+1),...
                                   xstartind(yind):xstartind(yind+1));
                tempAngGradient = angGradient(ystartind(xind):ystartind(xind+1),...
                                              xstartind(yind):xstartind(yind+1)); 
                tempAngGradient = tempAngGradient(edgeindex);
                
                sector(xind, yind).allAngGradients = tempAngGradient;
                [y bins] = hist(tempAngGradient, linspace(-pi, 0, numbins));
                
                % There are two primary directions which are perpendicular
                % in 3 dimensions.
                [~, ind2bins] = sort(y, 'descend');
%                 newind = find(abs(ind2bins-ind2bins(1))>3, 1, 'first');
%                 if isempty(newind)
%                     newind = find(abs(ind2bins-ind2bins(1))>2, 1, 'first');
%                     if isempty(newind)
%                         newind = find(abs(ind2bins-ind2bins(1))>1, 1, 'first');
%                     end
%                 end
%                 if ~isempty(newind)
%                     ind2bins([2 newind]) = ind2bins([newind 2]);
%                 end
                
                for b = 1:2
                    % Find the actual values that belong to this bin and
                    % take their mean.
                    sector(xind, yind).angGradient(b) = ...
                        mean(tempAngGradient(abs(tempAngGradient-bins(ind2bins(b))) <= 2*pi/(numbins + 1)));
                end
                
                sector(xind, yind).xpos = (xstartind(xind)+xstartind(xind+1))/2;
                sector(xind, yind).ypos = (ystartind(yind)+ystartind(yind+1))/2;

                t = sector(xind, yind);
                plot(ax1,...
                    sin(t.angGradient(1))*[-1*onoff 0 1]*quivlength + t.xpos,...
                    cos(t.angGradient(1))*[-1*onoff 0 1]*quivlength + t.ypos,...
                    'r-', 'linewidth', 3, 'xliminclude','off','yliminclude','off')
                plot(ax1,...
                    sin(t.angGradient(2))*[-1*onoff 0 1]*quivlength + t.xpos,...
                    cos(t.angGradient(2))*[-1*onoff 0 1]*quivlength + t.ypos,...
                    'g-', 'linewidth', 3, 'xliminclude','off','yliminclude','off')
                
                % Compute the cross-product of these two lines to get the
                % vector that points towards the vanishing point.
                vec = [cos(t.angGradient(:)) sin(t.angGradient(:)) ones(2,1)];
                z_cross = cross(vec(1,:), vec(2,:));
                sector(xind, yind).z_cross_ang = atan2(z_cross(2), z_cross(1));
                
                plot(ax1,...
                    sin(sector(xind, yind).z_cross_ang)*[-1*onoff 0 1]*quivlength + t.xpos,...
                    cos(sector(xind, yind).z_cross_ang)*[-1*onoff 0 1]*quivlength + t.ypos,...
                    'y-', 'linewidth', 3, 'xliminclude','off','yliminclude','off')
                
            end
        end
        drawnow

        % Create the A and b matrix
        A = zeros(numel(sector), 2);
        b = zeros(numel(sector), 1);
        count = 1;
        toDelete = [];
        for xind = 1:length(xstartind)-1
            for yind = 1: length(ystartind)-1
                t = sector(xind, yind);
                A(count, :) = [-cot(t.z_cross_ang) 1];
                b(count) = A(count, :)*[t.xpos t.ypos]';
                if isnan(A(count,1))
                    toDelete(end+1) = count;
                end
                count = count + 1;
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
        posterior = monteCarloTracker_step(prior, control, measurements);       
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


function posterior = monteCarloTracker_step(prior, control, measurement)
    pixelVar = 5; % # pixels

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
        newSamplesIndex = interp1(CDF, 1:numSamples, randn(trueNumSamples, 1), 'nearest', 'extrap');
        posterior.samples = prediction(newSamplesIndex, :) + pixelVar*randn(trueNumSamples, numDimensions);
        posterior.sampleIndex = newSamplesIndex;
        posterior.mean = mean(posterior.samples, 1);
        temp = bsxfun(@minus, posterior.mean, posterior.samples);
        posterior.var = std(temp(:));
        % Draw samples from the new distribution
%         posterior.samples = bsxfun(@plus,...
%             posterior.mean,...
%             posterior.var*randn(trueNumSamples, numDimensions));
    else
        % The weight for all samples is zero. I.E. the innovation is too
        % small to be usefull.
        % Resample using the current mean to create variations.
        disp('RESAMPLING')
        % Have to resample the distribution
        posterior.mean = mean(prediction, 1);
%         posterior.var = .0125;
%         posterior.var = .00125;
        posterior.var = pixelVar;
        posterior.samples = bsxfun(@plus, randn(numSamples, numDimensions)*posterior.var, posterior.mean);        
    end

end

function y = wrapToPi(x)
    % Makes x between [-pi pi]
    y = mod(x, sign(x)*2*pi);
    y(abs(y)>pi) = y(abs(y)>pi) - sign(y(abs(y)>pi))*2*pi;
end