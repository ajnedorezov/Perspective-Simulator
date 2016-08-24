%% Compare Obstacle classifier to ground truth

vid = VideoReader('External\seq05VD\0005VD.avi');

sampleRegionI = 290:320; %->
sampleRegionJ = 260:380; %v
intensityRange = 35;

myIPM = load('Examples\CamSeqManipulation\myIPM_smaller.mat');
myIPM = myIPM.myIPM;
nearestIPM = load('Examples\CamSeqManipulation\myIPM_smaller_nearest.mat');

mov = VideoWriter('Examples\CamSeqManipulation\PathIntersectingObstacles-seq05VD.avi');
open(mov);

invalidPixels = (myIPM.Weights{1}==0 & myIPM.Weights{1}==0 & myIPM.Weights{3}==0);

%%
hf = figure(1);

count = 1;
initializeSnake = true;
intersectionCounter = 1;
timeSteps = 7959:2:8159;
downRangeToObstacleOnPath = inf(1, length(timeSteps));
closestDownRangeToObstacle = inf(1, length(timeSteps));
while hasFrame(vid)
    if ~ishandle(hf)
        break
    end
    
    %% Grab the next video frame
%     imName = ['External\CamSeq01\0016E5_0' num2str(n)];
%     vidFrame= imread([imName '.png']);
%     vidFrameGT = imread([imName '_L.png']);
%     for n = 1:30
        vidFrame = readFrame(vid);
%     end
%     vidFrameGT = readFrame(vidGT);
    
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
        
%         channel = vidFrameGT(:,:,m);
%         ipmchannel = channel(nearestIPM.indices);
%         ipmchannel(nearestIPM.indices==1) = 0;
%         gtIPM_color(:,:,m) = ipmchannel;
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
    clf(figure(1)), 
    subplot(121), imshow(vidFrame)
%     title(sprintf('Current Frame: %d', n))
%     ylabel('Original')
    title('Original Frame:')
    
    %% Detect obstacles by checking if its a horizontal streak
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
    isObstacle(:,:,1) = ismember(newLabels, find(obstacles));

    
    ax = subplot(122); 
    tIm = imoverlay(uint8(rgbIPM), uint8(isObstacle), [1 0 0]);
    imshow(tIm, 'Parent', ax)
%     xlabel(ax, 'Cross Range (ft)')
%     ylabel(ax, 'Down Range (ft)')
    title(ax, 'Planning Result')
    set(ax,'yDir','normal','xdir','reverse')
    hold(ax, 'on')
    
    
%     r = vidFrameGT(:,:,1);
%     g = vidFrameGT(:,:,2);
%     b = vidFrameGT(:,:,3);
%     nonObstacle = r==128 & g==0 & b==192 |...   %LaneMkgsDriv
%                   r==128 & g==64 & b==128;      %Road
% 
%     labelIPM = double(nonObstacle(nearestIPM.indices));
    
%     gtObstacles = ~nonObstacle(nearestIPM.indices) & ~invalidPixels;
%     gtRoadway = nonObstacle(nearestIPM.indices) & ~invalidPixels;
    
    %% Do the maze path following algorithm stuff

    % Smooth the image and convert to an edge map
    smArea = 50;
    se = strel('ball', smArea, smArea);
    GI = imdilate(double(isObstacle), se)-smArea;
    f = smArea-GI;

    % Compute the GVF of the edge map f
    [px,py] = GVF(f, 0.2, 40);

    % Make the magnitude of all vectors equal
    magGVF = hypot(px,py) + 1e-10;
    px = px./magGVF;
    py = py./magGVF;

    % Make the obstacle have vectors towards the vanishing point
    imsize = size(px);

    [cc,rr] = meshgrid(1:imsize(2), 1:imsize(1));
    dy = vpy - rr;
    dx = vpx - cc;
    newMag =  sqrt(dx.*dx + dy.*dy) + eps;

    px(isObstacle)  = 0.75*px(isObstacle) + 0.25*dx(isObstacle)./newMag(isObstacle);
    py(isObstacle)  = 0.75*py(isObstacle) + 0.25*dy(isObstacle)./newMag(isObstacle);

%                 % Plot the gradient vectors
%                 [qx,qy] = meshgrid(1:10:imsize(1), 1:10:imsize(2));
%                 ind = sub2ind(imsize, qx,qy);
%                 subplot(122), quiver(qy,qx,px(ind),py(ind)); set(gca, 'ydir', 'normal','xdir','reverse')

    % Initialize the snake
    if initializeSnake 
        snakeTime = linspace(0,1, 100)';
        cx = floor(imsize(2)/2);
        cy = 1;
        snakeX = cx + snakeTime.*(vpx-cx); % vpx.*t + (1-t).*cx;
        snakeY = cy + snakeTime.*(vpy-cy);
        initializeSnake = false;
    else
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
    [snakeX,snakeY] = snakedeform(snakeX,snakeY,1,0.25,0.25,5,px,py,25);
    
    obstacleOnPath = interp2(double(isObstacle), snakeX, snakeY, 'nearest');
    obstacleIntersectionIndex = find(obstacleOnPath==1, 1, 'first');
    if ~isempty(obstacleIntersectionIndex)
        downRangeToObstacleOnPath(intersectionCounter) = snakeY(obstacleIntersectionIndex)*sy;

%         plot(ax, (xcenter-snakeX(obstacleIntersectionIndex))*sx, downRangeToObstacleOnPath(intersectionCounter), 'cx', 'markersize', 10, 'linewidth', 3);
        plot(ax, snakeX(obstacleIntersectionIndex), downRangeToObstacleOnPath(intersectionCounter)/sy, 'cx', 'markersize', 10, 'linewidth', 3);
    else
        downRangeToObstacleOnPath(intersectionCounter) = inf;
    end

    closestObstacle = isObstacle(:, 272);
    closestObstacleIntersectionIndex = find(closestObstacle==1, 1, 'first');
    if ~isempty(closestObstacleIntersectionIndex)
        closestDownRangeToObstacle(intersectionCounter) = closestObstacleIntersectionIndex*sy;
    end

    intersectionCounter = intersectionCounter + 1;
    
    
%     plot(ax, 0, 0, 'x', (vpx-xcenter)*sx, vpy*sy, 'o')
%     plot(ax, (xcenter-snakeX)*sx, snakeY*sy, 'Color', [0.5 1 0], 'linewidth', 2); 
    plot(ax, 0, 0, 'x', vpx, vpy, 'o')
    plot(ax, snakeX, snakeY, 'Color', [0.5 1 0], 'linewidth', 2);   
    
    drawnow
%     keyboard
    
%     break
    fprintf('Frame: %d\n', count)
    
    im = getframe(hf);
    writeVideo(mov, im.cdata);
    count = count + 1;
end
close(mov)

save('Examples\CamSeqManipulation\seq05VD_patheval_Results.mat', 'downRangeToObstacleOnPath', 'closestDownRangeToObstacle');
figure, plot(1:(intersectionCounter-1), downRangeToObstacleOnPath, 'bo', 1:(intersectionCounter-1), closestDownRangeToObstacle, 'ro')