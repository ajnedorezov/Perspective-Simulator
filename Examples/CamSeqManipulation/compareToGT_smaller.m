%% Compare Obstacle classifier to ground truth

vid = VideoReader('External\seq05VD\0005VD.avi');
vidGT = VideoReader('External\seq05VD\05VD_preview.mpg');
sampleRegionI = 290:320; %->
sampleRegionJ = 260:380; %v
intensityRange = 35;

imsize = size(readFrame(vid));
binaryIm = zeros(imsize(1), imsize(2), 3);

%%
if true
% if false
    myIPM = load('Examples\CamSeqManipulation\myIPM_smaller.mat');
    myIPM = myIPM.myIPM;
    nearestIPM = load('Examples\CamSeqManipulation\myIPM_smaller_nearest.mat');
else
    myIPM = IPM_vert(imsize(1:2),...
        'cameraZ', 7,...
        'theta', eps,...
        'stepSize', [.125 .25],...
        'xRange', [-50 50],...
        'yRange', [0 350]);
end

%%
hf = figure(1);

while hasFrame(vid)
    if ~ishandle(hf)
        break
    end
    
    %% Grab the next video frame
    for n = 1:30
        vidFrame = readFrame(vid);
    end
    vidFrameGT = readFrame(vidGT);

    %% Road surface removal
    for n = 1:3
        channel = vidFrame(:,:,n);
        roadRegion = channel(sampleRegionI, sampleRegionJ);
        avgPixelInt = mean(roadRegion(:));
        binaryIm(:,:,n) = channel < (avgPixelInt-intensityRange) | channel > (avgPixelInt+intensityRange);
    end
    ind = sum(binaryIm,3)==0;
    
    %% Perform the IPM transformation
    % Transform the image
    for n = 1:3
        channel = binaryIm(:,:,n);
        channel(ind) = 0;
        newVidFrame(:,:,n) = myIPM.performTransformation(double(channel));
        
        channel = vidFrame(:,:,n);
        rgbIPM(:,:,n) = myIPM.performTransformation(double(channel));
        
        channel = vidFrameGT(:,:,n);
        ipmchannel = channel(nearestIPM.indices);
        ipmchannel(nearestIPM.indices==1) = 0;
        gtIPM_color(:,:,n) = ipmchannel;
    end
%     grayIm = rgb2gray(newVidFrame);
%     newVidFrame = binaryIm;

    % Plot the current results
    clf(figure(1)), 
    subplot(121), imshow(vidFrame)
    title(sprintf('Current Time: %f', vid.CurrentTime))
    ylabel('Original')
    
    %% Detect obstacles by checking if its a horizontal streak
    grayIm = sum(newVidFrame,3) > 0;
    newLabels = bwlabeln(grayIm);

    % Get the region properties for the segments
    stats = regionprops(newLabels, 'BoundingBox', 'Extent', 'Orientation');

    % Decide if the cluster is streak like and something that
    % should be avoided
    obstacles = false(length(stats),1);
    for n = 1:length(stats)
        obstacles(n) = stats(n).BoundingBox(4) > 100 && stats(n).BoundingBox(4) > stats(n).BoundingBox(3) && stats(n).BoundingBox(3) > 30;
    end
%     obstacles = true(length(stats),1);
    
    % Plot the obstacles around the IPM image

    isObstacle(:,:,1) = uint8(ismember(newLabels, find(obstacles)));

    
    ax = subplot(122); 
    tIm = imoverlay(uint8(rgbIPM), isObstacle,[1 0 0]);
    imshow(tIm, 'Parent', ax)
    ylabel('IPM Road Surface Substracted')
    set(ax,'yDir','normal','xdir','reverse')
    
    
    r = vidFrameGT(:,:,1);
    g = vidFrameGT(:,:,2);
    b = vidFrameGT(:,:,3);
    nonObstacle = (r==128 & g==0 & b==192) |...   %LaneMkgsDriv
                  (r==128 & g==64 & b==128);      %Road

    labelIPM = double(nonObstacle(nearestIPM.indices));
    
    keyboard
    
    break
end
