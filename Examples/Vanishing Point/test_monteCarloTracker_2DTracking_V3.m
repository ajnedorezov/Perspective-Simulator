function test_monteCarloTracker_2DTracking_V3
    clc
    clear all
    close all
    movegui(figure(1), 'northwest')
    ax = axes;
    hPrior = line(nan, nan, 'Parent', ax, 'color', 'b', 'marker', 'o', 'linestyle', 'none');
    hPost = line(nan, nan, 'Parent', ax, 'color', 'r', 'marker', 'x', 'linestyle', 'none');
    hMeas = line(nan, nan, 'Parent', ax, 'color', 'c', 'marker', 's', 'linestyle', 'none');
    hGT = line(nan, nan, 'Parent', ax, 'color', 'c', 'linestyle', '-');
    hGT2 = line(nan, nan, 'Parent', ax, 'color', 'b', 'linestyle', '-');
    hTraj = line(nan, nan, 'Parent', ax, 'color', 'k', 'linestyle', '--');

    % Define some parameters
    numSamples = 200;
    numMeas = 5;
    numIterations = 200;
    
    % Initialize the filter with samples uniformly drawn from the square
    % [[0 1] [0 1]]
    prior.samples = rand(numSamples, 2);
    prior.mean = mean(prior.samples, 1);
    temp = bsxfun(@minus, prior.mean, prior.samples);
    prior.var = std(temp(:));%*eye(2);%diag(std(prior.samples, 0, 1));

    % Draw the initial estimates
    figure
    plot(prior.samples(:,1), prior.samples(:,2), 'co')
    title('Initialization of Monte Carlo Tracker')

    for t = 1:numIterations
        % Grab the measurements recieved by drawing from the true
        % trajectory of the particle. Make sure the ground truth is within
        % the search area.
        
        % Vertical Trajectory
%         gtPos = [0.5 0+t/numIterations];
        % Diagonal Trajectory
%         gtPos = [0+t/numIterations 0+t/numIterations];
        % Sinusoidal Trajectory
%         gtPos = [0.5+sin(2*pi*t/numIterations) 0+t/numIterations];
        % Figure 8 Trajectory
        gtPos(:,:,1) = .5*[cos(2*pi*t/numIterations) sin(4*pi*t/numIterations)]+.5;
%         gtPos(:,:,2) = .5*[cos(4*pi*t/numIterations+pi) sin(8*pi*t/numIterations+pi)]+.5;
        % Random Step
%         gtPos = gtPos + 0.01*randn(1,2);

        tempmeasurements = bsxfun(@plus, gtPos, 0.0125*randn(numMeas,2,2));
        tsize = size(tempmeasurements);
        measurements = zeros(tsize(1)*tsize(3), tsize(2));
        for m = 0:tsize(3)-1
            measurements(m*tsize(1) + (1:tsize(1)), :) = tempmeasurements(:,:,m+1);            
        end
        if t == 1
            % Properly initialize these
            control = zeros(size(gtPos));
            lastmeasurements = measurements;
            lastGTpos = gtPos;
        end
                
        posterior = monteCarloTracker_step(prior, control, measurements);

        set(hPrior, 'XData', prior.samples(:,1), 'YData', prior.samples(:,2))
        set(hPost, 'XData', posterior.samples(:,1), 'YData', posterior.samples(:,2))
        set(hMeas, 'XData',  measurements(:,1), 'YData', measurements(:,2))
        set(hGT, 'XData', [get(hGT, 'XData') gtPos(1,1,1)],...
                 'YData', [get(hGT, 'YData') gtPos(1,2,1)])
        if length(size(gtPos))==3
            set(hGT2, 'XData', [get(hGT2, 'XData') gtPos(1,1,2)],...
                      'YData', [get(hGT2, 'YData') gtPos(1,2,2)])
        end
        set(hTraj, 'XData', [get(hTraj, 'XData') posterior.mean(1)],...
                   'YData', [get(hTraj, 'YData') posterior.mean(2)])
        
        axis(ax, [-.5 1.5 -.5 1.5])
        
        drawnow
        
        % Update the control step by how much the particle has moved
        control = gtPos - lastGTpos + 0.00125*randn(size(lastGTpos));
        lastmeasurements = measurements;
        lastGTpos = gtPos;
        
        % The new posterior becomes the prior.
        prior = posterior;
    end
    if length(size(gtPos))==3
     legend(ax,'Prior PDF Samples', 'Posterior PDF Samples', 'Sensor Measurements',...
           'Ground Truth Position', 'Ground Truth Position 2', 'Tracker Position Estimate')
    else
       legend(ax,'Prior PDF Samples', 'Posterior PDF Samples', 'Sensor Measurements',...
           'Ground Truth Position', 'Tracker Position Estimate') 
    end
           
    % Compare the estimate postion to the actual position
    posgt = get(hGT, {'XData' 'YData'});
    posest = get(hTraj, {'XData' 'YData'});
    err = [posgt{1}; posgt{2}]-[posest{1}; posest{2}];
    err = hypot(err(1,:), err(2,:));
    figure
    plot(err)
    title('Error between position estimate and ground truth')
    xlabel('Step #')
    ylabel('Error')

end

function posterior = monteCarloTracker_step(prior, control, measurement)
    pixelVar = .0125;% 5;% # pixels % .00125;5;%

    [numSamples numDimensions] = size(prior.samples);
    trueNumSamples = numSamples;

    % Apply the control to each sample 
    tempPrediction = bsxfun(@plus, prior.samples, control);
    predsize = size(tempPrediction);
    if length(predsize) < 3
        predsize(3) = 1;
    end
    prediction = zeros(predsize(1)*predsize(3), predsize(2));
    for p = 0:predsize(3)-1
        prediction(p*predsize(1) + (1:predsize(1)), :) = tempPrediction(:,:,p+1);
    end
    numPrediction = size(prediction,1);

    % Compare the observation to our prediction. This is how different the
    % measurement and samples are.
    numMeas = size(measurement, 1);
    Innovation = nan(numPrediction, numDimensions, numMeas);
    for m = 1:numMeas
        Innovation(:,:,m) = bsxfun(@minus, measurement(m,:), prediction);
    end
    
    % Compute the new liklihood using gaussian distribution
    % exp(-1/2*(x-mu)'*sig^-1*(x-mu))
    L = ones(numPrediction, numMeas) / numSamples;
    varInv = 1/prior.var^2*eye(numDimensions);
%     varInv = 1/pixelVar^2*eye(numDimensions);
    for m = 1:numMeas
        for s = 1:numPrediction
            L(s,m) = exp(-0.5*Innovation(s,:,m)*varInv*Innovation(s,:,m)');
        end
    end
    L = mean(L, 2);
    
    % Update the distribution
    CDF = cumsum(L)/sum(L);
    if any(diff(CDF) == 0)
        inds = diff(CDF) == 0;
        CDF(inds) = [];
        prediction(inds,:) = [];
        numPrediction = size(prediction, 1);
    end
    
    if any(L)
        % Draw the samples from prediction distribution based on the
        % weights
        newSamplesIndex = interp1(CDF, 1:numPrediction, rand(trueNumSamples, 1), 'nearest', 'extrap');
        posterior.samples = prediction(newSamplesIndex, :) + pixelVar/10*randn(trueNumSamples, numDimensions);
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

