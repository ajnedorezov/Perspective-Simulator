function test_VPTracker

    clc
    clear all
    close all

    % Highway driving, changing lanes
%     a =  mmreader('C:\Program Files (x86)\MATLAB\R2011a Student\toolbox\vision\visiondemos\viplanedeparture.avi');

%     vidFrames = read(a);
%     vidFrames = a.frame;
    a.NumberOfFrames = 101;
    hFig1 = figure;
    ax1 = axes('Parent', hFig1);
%     hFig2 = figure;
%     ax2 = axes('Parent', hFig2);
    movegui(hFig1, 'northwest')
%     movegui(hFig2, 'southwest')

    % Initialize the video struct
%     mov = struct('cdata', [], 'colormap', []);
%     edges = struct('cdata', [], 'colormap', []);
    
    % Initialize the MC tracker
    numSamples = 1000;
    
    % Uniformly sample the size of the image
%     imsize = size(vidFrames(:,:,1,1));s
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

%         mov.cdata = vidFrames(:,:,:,k);
        fname = ['Test Images\CamSeq01\0016E5_0' num2str(2*(k-1) + 7959) '.png'];
        mov.cdata = imread(fname);
        mov.colormap = [];
        cla(ax1)
        imagesc(mov.cdata, 'Parent', ax1)

        % Perform edge detection
        greyim = double(rgb2gray(mov.cdata));
%         edges.cdata = edge(greyim, 'canny'); % More lines
        edges.cdata = edge(greyim, 'sobel'); % Less lines
        edges.colomap = [];

        imedge = edges.cdata;
    
        % Find the x & y derivatives
        xdir = imfilter(greyim, [-1 0 1]);
        ydir = imfilter(greyim, [-1 0 1]');
        immag = hypot(xdir, ydir);
        imang = atan2(ydir, xdir);

       % Draw the direction of the x & y derivatives for each of the edges.
        [xx, yy] = meshgrid(1:size(greyim,2), 1:size(greyim,1));
        figure(1)
        hold on
        quiver(xx(imedge), yy(imedge), xdir(imedge), ydir(imedge), 'color', 'b')

        % Break the image into a much coarser grid and average the x & y
        % derivates within that grid space;
        gridsize = 4;
        [sx, sy] = size(greyim);
        xstartind = round(linspace(1, sy, gridsize));
        ystartind = round(linspace(1, sx, gridsize));

        % Compute the average gradient for each grid space
        sector = struct;
        for xind = 1:length(xstartind)-1
            for yind = 1:length(ystartind)-1
                magGradient = immag(ystartind(xind):ystartind(xind+1),...
                                    xstartind(yind):xstartind(yind+1));
                angGradient = imang(ystartind(xind):ystartind(xind+1),...
                                    xstartind(yind):xstartind(yind+1));
                edges = imedge(ystartind(xind):ystartind(xind+1),...
                               xstartind(yind):xstartind(yind+1));
                sector(xind, yind).MAP = findManhattanAngles(magGradient, angGradient, edges);

                sector(xind, yind).xpos = (xstartind(xind)+xstartind(xind+1))/2;
                sector(xind, yind).ypos = (ystartind(yind)+ystartind(yind+1))/2;

                t = sector(xind, yind);
                quiver(t.xpos, t.ypos, 10*cos(t.MAP.angles(1)), 10*sin(t.MAP.angles(2)), 'color', 'r');
            end
        end

        % Create the A and b matrix
        A = zeros(numel(sector), 2);
        b = zeros(numel(sector), 1);
        count = 1;
        toDelete = [];
        for xind = 1:length(xstartind)-1
            for yind = 1: length(ystartind)-1
                t = sector(xind, yind);
                A(count, :) = [-cos(t.MAP.angles(1))/cos(t.MAP.angles(2)) 1];
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

        % Compute the intercept points
        pts = findVert(A, b);
        if isempty(pts)
            disp('No intersections found')
            continue
        else
            plot(pts(1,:), pts(2,:), 'cx')
            plot(mean(pts(1,:)), mean(pts(2,:)), 'mo')
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
        
        drawnow        
    catch ME
        disp(ME.message)
        disp(ME.stack(1).line)
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

function [MAP] = findManhattanAngles(magGradient, angGradient, edges)
    MAP = struct('likelihood', 0, 'P_edges', [], 'angles', [0 0 0]);
    numbins = 64;

    
    maxG = max(magGradient(:));
    if maxG > 0
        magGradient = magGradient/maxG;
    end
    counts = imhist(magGradient, numbins);
    y = cumsum(counts);
    y = y/max(y);
    y(diff(y)==0) = [];
    P_edge = interp1(y, linspace(1/(length(y)+1),1,length(y)), magGradient, 'neareast', 'extrap');
    
    % Only use the edges
    P_edge = P_edge(edges);
    angGradient = angGradient(edges);
    P_ang = 1/(2*pi)*ones([size(P_edge) 5]);
    if isempty(P_edge)
        return
    end

    % Do a coarse search
    alpha = (linspace(-40,40,10) - 90)*pi/180;
    beta = linspace(-40,40,10)*pi/180;
    gamma = (linspace(-10,10,3) - 90)*pi/180;
    
    MAP = AngleSearch(alpha, beta, gamma, P_edge, P_ang, angGradient, MAP);
        
    % Do a finer search
    tpi = 10/pi;
    alpha = linspace(MAP.angles(1)-tpi, MAP.angles(1)+tpi, 10);
    beta = linspace(MAP.angles(2)-tpi, MAP.angles(2)+tpi, 10);
    gamma = linspace(MAP.angles(3)-tpi/4, MAP.angles(3)+tpi/4, 3);
    
    MAP = AngleSearch(alpha, beta, gamma, P_edge, P_ang, angGradient, MAP);
    
end

function MAP = AngleSearch(alpha, beta, gamma, P_edge, P_ang, angGradient, MAP)
    m_u = [0.02 0.02 0.02 0.04 0.9];
    m_u = reshape(m_u, [1 1 numel(m_u)]);
    c = 0.1;
    tau = 6*pi/180;
    
    for a = alpha
        P_ang(:,:,1:3) = c/(2*pi-4*tau);
        angdiff_alpha = abs(wrapToPi(angGradient - a));
        indalpha = angdiff_alpha <= tau;
        angdiff_alpha = abs(wrapToPi(angGradient - a - pi));
        indalpha = indalpha | angdiff_alpha <= tau;
        temp = P_ang(:,:,1);
        temp(indalpha) = (1-c)/(4*tau);
        P_ang(:,:,1) = temp;
        
        for b = beta
            angdiff_beta = abs(wrapToPi(angGradient - b));
            indbeta = angdiff_beta <= tau;
            angdiff_beta = abs(wrapToPi(angGradient - b - pi));
            indbeta = indbeta | angdiff_beta <= tau;
            temp = P_ang(:,:,2);
            temp(indbeta) = (1-c)/(4*tau);
            P_ang(:,:,2) = temp;
            
            for c = gamma
                angdiff_gamma = abs(wrapToPi(angGradient - c));
                indgamma = angdiff_gamma <= tau;
                angdiff_gamma = abs(wrapToPi(angGradient - c - pi));
                indgamma = indgamma | angdiff_gamma <= tau;
                temp = P_ang(:,:,3);
                temp(indgamma) = (1-c)/(4*tau);
                P_ang(:,:,3) = temp;
                
                % Compute the liklihood
                P_onoff = bsxfun(@times, P_edge, P_ang);
                P_e_mu_etc = bsxfun(@times, P_onoff, m_u);

                likelihood = log10(sum(P_e_mu_etc, 3));
                likelihood = sum(likelihood(:));
                
                if likelihood < MAP.likelihood
                    MAP.likelihood = likelihood;
                    MAP.P_edges = P_e_mu_etc;
                    MAP.angles = [a b c];
                end
            end
        end
    end
end

function y = wrapToPi(x)
    % Makes x between 0 & 2*pi
    y = mod(x, sign(x)*2*pi);
    y(abs(y)>pi) = y(abs(y)>pi) - sign(y(abs(y)>pi))*2*pi;
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
        newSamplesIndex = interp1(CDF, 1:numSamples, randn(numSamples, 1), 'nearest', 'extrap');
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

