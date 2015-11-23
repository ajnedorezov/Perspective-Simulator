function test_monteCarloTracker_2DTracking_V2
    clc
    clear all
    close all
    movegui(figure(1), 'northwest')
    ax = axes;
    hPrior = line(nan, nan, 'Parent', ax, 'color', 'b', 'marker', 'o', 'linestyle', 'none');
    hPost = line(nan, nan, 'Parent', ax, 'color', 'r', 'marker', 'x', 'linestyle', 'none');
    hMeas = line(nan, nan, 'Parent', ax, 'color', 'c', 'marker', 's', 'linestyle', 'none');
    hGT = line(nan, nan, 'Parent', ax, 'color', 'c', 'linestyle', '-');
    hTraj = line(nan, nan, 'Parent', ax, 'color', 'k', 'linestyle', '--');
    % Define some parameters
    numSamples = 100;
    numMeas = 5;
    numIterations = 1000;
    
    % Initialize the filter with samples uniformly drawn from the square
    % [[0 1] [0 1]]
    prior.samples = rand(numSamples, 2);
    prior.mean = mean(prior.samples, 1);
    temp = bsxfun(@minus, prior.mean, prior.samples);
    prior.var = std(temp(:));%*eye(2);%diag(std(prior.samples, 0, 1));

    control = [0 0];
%     control = 0;
%     lastmeasurements = [0 0];
%     lastGTpos = [0 0];
    for t = 1:numIterations
        % Grab the measurements recieved by drawing from the true
        % trajectory of the particle
%         gtPos = [0.5 0+t/numIterations];
%         gtPos = [0.5+.5*t/numIterations 0+t/numIterations];
        gtPos = [0.5+sin(2*pi*t/numIterations) 0+t/numIterations];

        measurements = bsxfun(@plus, gtPos, 0.0125*randn(numMeas,2));
        if t == 1
            % Properly initialize this
            lastmeasurements = measurements;
            lastGTpos = gtPos;
        end
                
        posterior = monteCarloTracker_step(prior, control, measurements);
%         cla
%         if isfield(posterior, 'sampleIndex')
%             indexedSamples = prior.samples(posterior.sampleIndex, :);
%         else
%             indexedSamples = nan(1,2);
%         end
%         plot(ax,...
%             prior.samples(:,1), prior.samples(:,2), 'o',...
%             indexedSamples(:,1), indexedSamples(:,2), 'ms',...
%             posterior.samples(:,1), posterior.samples(:,2), 'rx', ...
%             measurements(:,1), measurements(:,2), 'cs',...
%             gtPos(:,1), gtPos(:,2), 'c.');
        set(hPrior, 'XData', prior.samples(:,1), 'YData', prior.samples(:,2))
        set(hPost, 'XData', posterior.samples(:,1), 'YData', posterior.samples(:,2))
        set(hMeas, 'XData',  measurements(:,1), 'YData', measurements(:,2))
        set(hGT, 'XData', [get(hGT, 'XData') gtPos(1)],...
                 'YData', [get(hGT, 'YData') gtPos(2)])
        set(hTraj, 'XData', [get(hTraj, 'XData') posterior.mean(1)],...
                   'YData', [get(hTraj, 'YData') posterior.mean(2)])
        
        
        axis(ax, [-.5 1.5 -.5 1.5])
        
        drawnow
        
        % Update the control step by how much the particle has moved
        control = gtPos - lastGTpos;%[0 1/numIterations];
%         control = measurements - lastmeasurements;%[0 1/numIterations];
%         control = norm(control);
        lastmeasurements = measurements;
        lastGTpos = gtPos;
        
        % The new posterior becomes the prior.
        prior = posterior;
    end
    
    % Compare the estimate postion to the actual position
    posgt = get(hGT, {'XData' 'YData'});
    posest = get(hTraj, {'XData' 'YData'});
    err = [posgt{1}; posgt{2}]-[posest{1}; posest{2}];
    err = hypot(err(1,:), err(2,:));
    figure(2)
    plot(err)

end


function posterior = monteCarloTracker_step(prior, control, measurement)

    [numSamples numDimensions] = size(prior.samples);
    trueNumSamples = numSamples;

    % Apply the control to each sample 
    prediction = bsxfun(@plus, prior.samples, control);
%     % Make the prediction step a uniform circle
%     stepangle = 2*pi*rand(numSamples, 1);
%     prediction = prior.samples + control*[cos(stepangle) sin(stepangle) ones(numSamples, max(0,numDimensions-2))];
    
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
%         L(s) = exp(-.5*norm(Innovation));
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
        posterior.samples = prediction(newSamplesIndex, :);
        posterior.sampleIndex = newSamplesIndex;
        posterior.mean = mean(posterior.samples, 1);
        temp = bsxfun(@minus, posterior.mean, posterior.samples);
        posterior.var = std(temp(:));
        % Draw samples from the new distribution
        posterior.samples = bsxfun(@plus,...
            posterior.mean,...
            posterior.var*randn(trueNumSamples, numDimensions));
    else
        % The weight for all samples is zero. I.E. the innovation is too
        % small to be usefull.
        % Resample using the current mean to create variations.
        disp('RESAMPLING')
        % Have to resample the distribution
        posterior.mean = mean(prediction, 1);
        posterior.var = .0125;
%         posterior.var = .00125;
        posterior.samples = bsxfun(@plus, randn(numSamples, numDimensions)*posterior.var, posterior.mean);        
    end

end