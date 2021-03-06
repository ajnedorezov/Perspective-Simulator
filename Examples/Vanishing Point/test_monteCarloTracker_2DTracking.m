function test_monteCarloTracker
    clc
    clear all
    close all
    movegui(figure(1), 'northwest')
    ax = axes;
    % Define some parameters
    numSamples = 100;
    numIterations = 100;
    
    % Initialize the filter with samples uniformly drawn from the square
    % [[0 1] [0 1]]
    prior.samples = rand(numSamples, 2);
    prior.mean = mean(prior.samples, 1);
    temp = bsxfun(@minus, prior.mean, prior.samples);
    prior.var = std(temp(:));%*eye(2);%diag(std(prior.samples, 0, 1));

    control = [0 0];
%     control = 0;
    lastmeasurements = [0 0];
    for t = 1:numIterations
        % Grab the measurements recieved by drawing from the true
        % trajectory of the particle
        gtPos = [0.5+.5*t/numIterations 0+t/numIterations];
        measurements =  gtPos;% + 0.025*randn(1,2);
        if t == 1
            % Properly initialize this
            lastmeasurements = measurements;
        end
                
        posterior = monteCarloTracker_step(prior, control, measurements);
        cla
        if isfield(posterior, 'sampleIndex')
            indexedSamples = prior.samples(posterior.sampleIndex, :);
        else
            indexedSamples = nan(1,2);
        end
        plot(ax,...
            prior.samples(:,1), prior.samples(:,2), 'o',...
            indexedSamples(:,1), indexedSamples(:,2), 'ms',...
            posterior.samples(:,1), posterior.samples(:,2), 'rx', ...
            measurements(:,1), measurements(:,2), 'cs',...
            gtPos(:,1), gtPos(:,2), 'c.');
        
        
        axis(ax, [-.5 1.5 -.5 1.5])
        
        drawnow
        
        % Update the control step by how much the particle has moved
        control = measurements - lastmeasurements;%[0 1/numIterations];
%         control = norm(control);
        lastmeasurements = measurements;
        
        % The new posterior becomes the prior.
        prior = posterior;
    end

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
    % measurement and samples are 
    Innovation = bsxfun(@minus, measurement, prediction);
    
    % Compute the new liklihood using gaussian distribution
    % exp(-1/2*(x-mu)'*sig^-1*(x-mu))
    L = ones(numSamples, 1) / numSamples;
    varInv = 1/prior.var^2*eye(numDimensions);
    for s = 1:numSamples
        L(s) = exp(-0.5*Innovation(s,:)*varInv*Innovation(s,:)');
%         L(s) = exp(-.5*norm(Innovation));
    end
    
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
        disp('RESAMPLING')
        % Have to resample the distribution
        posterior.mean = mean(prediction, 1);
        posterior.var = .0125;%*eye(numDimensions);% diag(std(prediction, 0 , 1));
        posterior.samples = bsxfun(@plus, randn(numSamples, numDimensions)*posterior.var, posterior.mean);        
    end

end