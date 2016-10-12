%% Compare Obstacle classifier to ground truth

sampleRegionI = 580:650; %->
sampleRegionJ = 360:560; %v
intensityRange = 35;

myIPM = load('Examples\CamSeqManipulation\myIPM.mat');
myIPM = myIPM.myIPM;
nearestIPM = load('Examples\CamSeqManipulation\myIPM_nearest.mat');

invalidPixels = (myIPM.Weights{1}==0 & myIPM.Weights{1}==0 & myIPM.Weights{3}==0);

vid = VideoWriter('Thesis Images\Chapter 4\ObstacleDetector_CamSeq01_Results.avi');
open(vid);

%%
hf = figure(1);
set(hf, 'Position', [100 100 1200 700]); 

count = 1;
for n = 7959:2:8159
    if ~ishandle(hf)
        break
    end
    
    %% Grab the next video frame
    imName = ['External\CamSeq01\0016E5_0' num2str(n)];
    vidFrame= imread([imName '.png']);
    vidFrameGT = imread([imName '_L.png']);
    
    imsize = size(vidFrame);
    binaryIm = zeros(imsize(1), imsize(2), 3);

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
    subplot(131), imshow(vidFrame)
%     title(sprintf('Current Image: %s', imName))
    title('Original Image')
    ax = subplot(132); imshow(uint8(rgbIPM));
    title('IPM Transformed Image')
    set(ax,'yDir','normal','xdir','reverse')
    
    %% Detect obstacles by checking if its a horizontal streak
    h = fspecial('gaussian', [10 10]);
    newVidFrame = imfilter(newVidFrame, h);
    
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

    
    
    ax = subplot(133); 
    tIm = imoverlay(uint8(rgbIPM), isObstacle,[1 0 0]);
    imshow(tIm, 'Parent', ax)
    title('Detected Obstacles')
    set(ax,'yDir','normal','xdir','reverse')
    
    im = getframe(hf);
    writeVideo(vid, im.cdata);
    
    
%     hsv = rgb2hsv(double(vidFrameGT));
%     h = double(hsv(:,:,1));
%     s = double(hsv(:,:,2));
%     v = double(hsv(:,:,3));
%     nonObstacle = (h >= 0.76 & h < 0.8) |...   %LaneMkgsDriv
%                   (h >= 0.8 & h < 0.93);      %Road
% 	nonObstacle = imdilate(nonObstacle, true(5,5));
%     nonObstacle = ~imdilate(~nonObstacle, true(10,10));
%               
%     labelIPM = double(nonObstacle(nearestIPM.indices));
%     
    
    
%     keyboard
    
%     break
    fprintf('Frame: %s\n', imName)
    drawnow
end
close(vid)
