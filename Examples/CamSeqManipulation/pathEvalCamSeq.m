%% Compare Obstacle classifier to ground truth

sampleRegionI = 580:650; %->
sampleRegionJ = 360:560; %v
intensityRange = 35;

myIPM = load('Examples\CamSeqManipulation\myIPM.mat');
myIPM = myIPM.myIPM;
nearestIPM = load('Examples\CamSeqManipulation\myIPM_nearest.mat');

drOffset = 40;

saveDir = 'Thesis Images\Chapter 5\Section 5.3.2\';
% saveName = 'CamSeqMain';
% useGTObstacles = false;
saveName = 'CamSeqMainUsingGT';
useGTObstacles = true;

makeVideo = true;
% makeVideo = false;
if makeVideo
    mov = VideoWriter([saveDir saveName '-DualPlot.avi']);
    set(mov, 'FrameRate', 4)
    open(mov);
    mov2 = VideoWriter([saveDir saveName '.avi']);
    set(mov2, 'FrameRate', 4)
    open(mov2);
end

invalidPixels = (myIPM.Weights{1}==0 & myIPM.Weights{1}==0 & myIPM.Weights{3}==0);

%%
hf = figure(1);
hf2 = figure(2);

count = 1;
initializeSnake = true;
intersectionCounter = 1;
timeSteps = 7959:2:8159;
downRangeToObstacleOnPath = inf(1, length(timeSteps));
closestDownRangeToObstacle = inf(1, length(timeSteps));
for n = timeSteps
    if ~ishandle(hf) || ~ishandle(hf2)
        break
    end
    
    %% Grab the next video frame
    imName = ['External\CamSeq01\0016E5_0' num2str(n)];
    vidFrame= imread([imName '.png']);
    vidFrameGT = imread([imName '_L.png']);
    
    imsize = size(vidFrame);
    binaryIm = zeros(imsize(1), imsize(2), 3);

     %% Update the VP Tracker
     if initializeSnake
         myVPTracker = VPTracker(1000, imsize(1:2));
     end
     control = [0 0];%[delT self.RoadParams.laneWidth*sin(t/25) + 3*self.RoadParams.laneWidth/2];
     myVPTracker.Update(vidFrame, control);
    
    %% Road surface removal
    for m = 1:3
        channel = vidFrame(:,:,m);
        roadRegion = channel(sampleRegionI, sampleRegionJ);
        avgPixelInt = mean(roadRegion(:));
        binaryIm(:,:,m) = channel < (avgPixelInt-intensityRange) | channel > (avgPixelInt+intensityRange);
    end
    ind = sum(binaryIm,3)==0;
    
    %% Perform the IPM transformation
    % Transform the image
    for m = 1:3
        channel = binaryIm(:,:,m);
        channel(ind) = 0;
        newVidFrame(:,:,m) = myIPM.performTransformation(double(channel));
        
        channel = vidFrame(:,:,m);
        rgbIPM(:,:,m) = myIPM.performTransformation(double(channel));
        
        channel = vidFrameGT(:,:,m);
        ipmchannel = channel(nearestIPM.indices);
        ipmchannel(nearestIPM.indices==1) = 0;
        gtIPM_color(:,:,m) = ipmchannel;
    end
%     grayIm = rgb2gray(newVidFrame);
%     newVidFrame = binaryIm;

    % Transform the vanishing point
    ipmVP = myIPM.transformSinglePoint(max(myVPTracker.prior.mean(2), myIPM.rHorizon), myVPTracker.prior.mean(1));

    % Get the current vanishing point estimate
    origvpx = (ipmVP(1)-myIPM.xRange(1))*size(newVidFrame,2)/diff(myIPM.xRange);
    origvpy = (ipmVP(2)-myIPM.yRange(1))*size(newVidFrame,1)/diff(myIPM.yRange);

    % Limit vp to point in image
    m = (origvpx-size(newVidFrame,2)/2)/origvpy;
    vpy = size(newVidFrame,1);
    vpx = vpy*m + size(newVidFrame,2)/2;

    % Plot the current results
    clf(hf), 
    ax0 = subplot(3,1,1,'Parent', hf); imshow(vidFrame, 'Parent', ax0);
%     title(sprintf('Current Frame: %d', n))
%     ylabel('Original')
    ylabel(ax0, 'Original Frame:')
    
    %% Get the GT labeling of obstacles
    r = vidFrameGT(:,:,1);
    g = vidFrameGT(:,:,2);
    b = vidFrameGT(:,:,3);
    nonObstacle = (r==128 & g==0 & b==192) |...   %LaneMkgsDriv
                  (r==128 & g==64 & b==128);      %Road

    labelIPM = double(nonObstacle(nearestIPM.indices));
    
    gtObstacles = ~nonObstacle(nearestIPM.indices) & ~invalidPixels;
%     gtRoadway = nonObstacle(nearestIPM.indices) & ~invalidPixels;
    
    %% Detect obstacles by checking if its a horizontal streak
    % Filter the image to get rid of some noise
    h = fspecial('gaussian', [10 10]);
    newVidFrame = imfilter(newVidFrame, h);
        
    % Convert the image to binary
    grayIm = sum(newVidFrame,3) > 0;
    newLabels = bwlabeln(grayIm);

    % Get the region properties for the segments
    stats = regionprops(newLabels, 'BoundingBox', 'Extent', 'Orientation');

    % Decide if the cluster is streak like and something that
    % should be avoided
    obstacles = false(length(stats),1);
    for m = 1:length(stats)
        obstacles(m) = stats(m).BoundingBox(4) > 100 && stats(m).BoundingBox(4) > stats(m).BoundingBox(3) && stats(m).BoundingBox(3) > 30;
    end
%     obstacles = true(length(stats),1);
    
    % Plot the obstacles around the IPM image
    if useGTObstacles
%         isObstacle(:,:,1) = imdilate(gtObstacles, true(1,35));
        isObstacle(:,:,1) = imdilate(gtObstacles, true(1,5));
    else
%         isObstacle(:,:,1) = ismember(newLabels, find(obstacles));
        isObstacle(:,:,1) = imdilate(ismember(newLabels, find(obstacles)), true(1,35));
        
    end
    
    %% Plot the obstacle data
    ax = subplot(3,1,2:3, 'Parent', hf); 
    tIm = imoverlay(uint8(rgbIPM), uint8(isObstacle), [1 0 0]);
    imshow(rot90(tIm,2), 'Parent', ax, 'XData', myIPM.xRange, 'YData', fliplr(myIPM.yRange)-drOffset); hold(ax, 'on')
    axis(ax, 'equal', 'tight', 'on');
    xlabel(ax, 'Cross Range (ft)')
    ylabel(ax, {'Planning Result' 'Down Range (ft)'})
%     title(ax, 'Planning Result')
    set(ax,'yDir','normal')
    ylim(ax, [0 310])
    
    clf(hf2)
    ax2 = gca(hf2);
    cla(ax2);
    imshow(rot90(tIm,2), 'Parent', ax2, 'XData', myIPM.xRange, 'YData', fliplr(myIPM.yRange)-drOffset); hold(ax2, 'on')
    axis equal tight on
%     set(ax2,'yDir','reverse','xdir','normal')
    set(ax2, 'ydir', 'normal')
    ylabel(ax2, 'Down Range (ft)')
    xlabel(ax2, 'Cross Range (ft)')
    ylim(ax2, [0 310])
    
    %% Do the maze path following algorithm stuff

    % Smooth the image and convert to an edge map
    smArea = 50;
    se = strel('ball', smArea, smArea);
    GI = imdilate(double(isObstacle), se)-smArea;
    f = smArea-GI;

    % Compute the GVF of the edge map f
    [px,py] = GVF(f, 0.2, 40);

    imsize = size(px);
%     % Make the magnitude of all vectors equal
%     magGVF = hypot(px,py) + 1e-10;
%     px = px./magGVF;
%     py = py./magGVF;
% 
%     % Make the obstacle have vectors towards the vanishing point
%     imsize = size(px);
% 
%     [cc,rr] = meshgrid(1:imsize(2), 1:imsize(1));
%     dy = vpy - rr;
%     dx = vpx - cc;
%     newMag =  sqrt(dx.*dx + dy.*dy) + eps;
% 
%     px(isObstacle)  = 0.75*px(isObstacle) + 0.25*dx(isObstacle)./newMag(isObstacle);
%     py(isObstacle)  = 0.75*py(isObstacle) + 0.25*dy(isObstacle)./newMag(isObstacle);

    % Apply a clockwise/counterclockwise rotation around the edges
    LeftRight = [zeros(size(f,1),1) diff(f, [], 2)];
    UpDown = [zeros(1,size(f,2)); diff(f, [], 1)];
    maxEdge = 1;

%     clockwise = true;
    clockwise = false;
    if clockwise 
%     if (posvec(2) > 4*self.RoadParams.laneWidth/2)
        py(LeftRight < 0) = -maxEdge/2;    px(LeftRight < 0) = 0;
        py(LeftRight > 0) = maxEdge/2;     px(LeftRight > 0) = 0;
        py(UpDown < 0) = 0;
        py(UpDown > 0) = 0;
        px(UpDown < 0) = maxEdge/2;
        px(UpDown > 0) = -maxEdge/2;
    else
        py(LeftRight < 0) = maxEdge/2;    px(LeftRight < 0) = 0;
        py(LeftRight > 0) = -maxEdge/2;     px(LeftRight > 0) = 0;
        py(UpDown < 0) = 0;
        py(UpDown > 0) = 0;
        px(UpDown < 0) = -maxEdge/2;
        px(UpDown > 0) = maxEdge/2;
    end    

    % Apply a slope towards the goal point
    towardsGoal = true;
%                 towardsGoal = false;
    [cc,rr] = meshgrid(1:imsize(2), 1:imsize(1));
    if towardsGoal
        % Need to flip vpx due to plotting
%         dx = imsize(2) - vpx - cc;
%         dy = vpy - rr;
        dx = vpx - cc;
        dy = vpy - rr;
    else
%         dx = cc - (imsize(2) - vpx);
%         dy = rr - vpy;
        dx = cc - vpx;
        dy = rr - vpy;
    end
    newMag =  sqrt(dx.*dx + dy.*dy) + eps;
    newMag = 1 - newMag./max(max(newMag));
    [fx2,fy2] = gradient(newMag);
    mag = hypot(fx2,fy2) + eps;
    fx2 = fx2./mag;
    fy2 = fy2./mag;

    pmag = max(max(hypot(px,py)));
    px = px/pmag;
    py = py/pmag;

    px(isObstacle) = fx2(isObstacle);
    py(isObstacle) = fy2(isObstacle);

    % Make the magnitude of all vectors equal
    magGVF = hypot(px,py) + 1e-10;
    px = px./magGVF;
    py = py./magGVF;
    px(1:(4*drOffset),:) = 0;
    py(1:(4*drOffset),:) = 0;

%     % Plot the gradient vectors
%     figure
%     [qx,qy] = meshgrid(1:5:imsize(1), 1:5:imsize(2));
%     ind = sub2ind(imsize, qx,qy);
%     quiver(qy,qx,px(ind),py(ind)); set(gca, 'ydir', 'normal','xdir','reverse'), axis equal

    % Initialize the snake
    if initializeSnake 
        snakeTime = linspace(0,1, 100)';
        cx = floor(imsize(2)/2);
        cy = 4*drOffset;
        snakeX = cx + snakeTime.*(vpx-cx); % vpx.*t + (1-t).*cx;
        snakeY = cy + snakeTime.*(vpy-cy);
        initializeSnake = false;
    else
        snakeX(1) = floor(imsize(2)/2);    
        snakeY(1) = 4*drOffset;
        snakeX(end) = vpx;
        snakeY(end) = vpy;
    end
    xcenter = size(newVidFrame,2)/2;
    sx = diff(myIPM.xRange)/size(newVidFrame,2);
    sy = diff(myIPM.yRange)/size(newVidFrame,1);
    
% 
%                 [snakeX, snakeY] = snakedeform(x,y,1,0.75,0.5,25,px,py,5*5);
%                 [snakeX,snakeY] = snakedeform(x,y,1,0.75,0.25,25,px,py,5*20);
%                 [snakeX,snakeY] = snakedeform(snakeX,snakeY,1,0.75,0.25,20,px,py,25); %*
    [snakeX,snakeY] = snakedeform(snakeX,snakeY,1,0.25,0.25,5,px,py,50);
    
    obstacleOnPath = interp2(double(isObstacle), snakeX, snakeY, 'nearest');
    obstacleIntersectionIndex = find(obstacleOnPath==1, 1, 'first');
    if ~isempty(obstacleIntersectionIndex)
        downRangeToObstacleOnPath(intersectionCounter) = snakeY(obstacleIntersectionIndex)*sy - drOffset;

        plot(ax, (xcenter-snakeX(obstacleIntersectionIndex))*sx, downRangeToObstacleOnPath(intersectionCounter), 'cx', 'markersize', 10, 'linewidth', 3);
        plot(ax2, (xcenter-snakeX(obstacleIntersectionIndex))*sx, downRangeToObstacleOnPath(intersectionCounter), 'cx', 'markersize', 10, 'linewidth', 3);
%         plot(ax, snakeX(obstacleIntersectionIndex), downRangeToObstacleOnPath(intersectionCounter)/sy, 'cx', 'markersize', 10, 'linewidth', 3);
%         plot(ax2, snakeX(obstacleIntersectionIndex), downRangeToObstacleOnPath(intersectionCounter)/sy, 'cx', 'markersize', 10, 'linewidth', 3);
    else
        downRangeToObstacleOnPath(intersectionCounter) = inf;
    end

%     closestObstacle = isObstacle(:, 272);
    closestObstacle = gtObstacles(:, round(xcenter));
    closestObstacleIntersectionIndex = find(closestObstacle==1, 1, 'first');
    if ~isempty(closestObstacleIntersectionIndex)
        closestDownRangeToObstacle(intersectionCounter) = closestObstacleIntersectionIndex*sy-drOffset;
        plot(ax, 0, closestDownRangeToObstacle(intersectionCounter), 'mx', 'markersize', 10, 'linewidth', 3);
        plot(ax2, 0, closestDownRangeToObstacle(intersectionCounter), 'mx', 'markersize', 10, 'linewidth', 3);
    end

    intersectionCounter = intersectionCounter + 1;
    
    %% Convert the snake into a commanded heading
    
    % Grab a point ~50ft in down the curve
    drInd = interp1(snakeY*sy, 1:length(snakeY), 50, 'nearest');
    if isempty(drInd) || isnan(drInd)
        % rough estimate: 50ft/300ft*100pts --> 17th index
        drInd = 17;
    end
    ptX = (xcenter - snakeX(drInd))*sx;
    ptY = snakeY(drInd)*sy;
    commandedHeading = atan2(ptX,ptY);

    
%     plot(ax, 0, 0, 'x', (vpx-xcenter)*sx, vpy*sy, 'o')
%     plot(ax, (xcenter-snakeX)*sx, snakeY*sy, 'Color', [0.5 1 0], 'linewidth', 2); 
    plot(ax, (xcenter-snakeX)*sx, snakeY*sy-drOffset, 'Color', [0.5 1 0], 'linewidth', 2); 
%     plot(ax, 0, 0, 'x', (vpx-xcenter)*sx, vpy*sy, 'o')
%     quiver(0,0, ptX, ptY, 'c', 'Parent', ax, 'LineWidth', 3)
    
    ax2 = gca(hf2);
    plot(ax2, (xcenter-snakeX)*sx, snakeY*sy-drOffset, 'Color', [0.5 1 0], 'linewidth', 2); 
%     plot(ax2, 0, 0, 'x', (vpx-xcenter)*sx, vpy*sy, 'o')
%     quiver(0,0, ptX, ptY, 'c', 'Parent', ax2, 'LineWidth', 3)
    
    drawnow
%     keyboard
    
%     break
    fprintf('Frame: %d\n', n)
    ylim(ax, [0 310])
    ylim(ax2, [0 310])
    if makeVideo
        im = getframe(hf);
        writeVideo(mov, im.cdata);
        im = getframe(hf2);
        writeVideo(mov2, im.cdata);
    end
end
if makeVideo
    close(mov)
    save([saveDir saveName '_Results.mat'], 'intersectionCounter', 'downRangeToObstacleOnPath', 'closestDownRangeToObstacle');
end


figure, plot(1:(intersectionCounter-1), downRangeToObstacleOnPath, 'bo', 1:(intersectionCounter-1), closestDownRangeToObstacle, 'ro')