%% Compare Obstacle classifier to ground truth

sampleRegionI = 580:650; %->
sampleRegionJ = 360:560; %v
intensityRange = 35;

myIPM = load('Examples\CamSeqManipulation\myIPM.mat');
myIPM = myIPM.myIPM;
nearestIPM = load('Examples\CamSeqManipulation\myIPM_nearest.mat');

invalidPixels = (myIPM.Weights{1}==0 & myIPM.Weights{1}==0 & myIPM.Weights{3}==0);

%%
hf = figure(1);

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
    subplot(121), imshow(vidFrame)
    title(sprintf('Current Image: %s', imName))
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
    nonObstacle = r==128 & g==0 & b==192 |...   %LaneMkgsDriv
                  r==128 & g==64 & b==128;      %Road

    labelIPM = double(nonObstacle(nearestIPM.indices));
    
    gtObstacles = ~nonObstacle(nearestIPM.indices) & ~invalidPixels;
    gtRoadway = nonObstacle(nearestIPM.indices) & ~invalidPixels;
    
    
    se = strel('ball',10,10);
    BW1 = edge(rgb2ind(vidFrameGT,256), 'canny');
    BW2 = imdilate(double(BW1),se);
    edges = BW2 > 10.5;
    edgePixels = edges(nearestIPM.indices);
    
    % obstacle correctly identified as obstacle
    truePositive(count) = sum(sum(gtObstacles & isObstacle));% / sum(gtObstacles(:));
    % roadway incorrectly identified as obstacle
    falsePositive(count) = sum(sum(gtObstacles & ~isObstacle));% / sum(gtObstacles(:));
    % roadway correctly identified as roadway
    trueNegative(count) = sum(sum(gtRoadway & ~isObstacle));% / sum(gtRoadway(:));
    % obstacle incorrectly identified as roadway
    falseNegative(count) = sum(sum(gtRoadway & isObstacle));% / sum(gtRoadway(:));

%     % obstacle correctly identified as obstacle
%     truePositive(count) = sum(sum(gtObstacles(edgePixels) & isObstacle(edgePixels)));% / sum(gtObstacles(:));
%     % roadway incorrectly identified as obstacle
%     falsePositive(count) = sum(sum(gtObstacles(edgePixels) & ~isObstacle(edgePixels)));% / sum(gtObstacles(:));
%     % roadway correctly identified as roadway
%     trueNegative(count) = sum(sum(gtRoadway(edgePixels) & ~isObstacle(edgePixels)));% / sum(gtRoadway(:));
%     % obstacle incorrectly identified as roadway
%     falseNegative(count) = sum(sum(gtRoadway(edgePixels) & isObstacle(edgePixels)));% / sum(gtRoadway(:));
% 
    sensitivity(count) = truePositive(count) / (truePositive(count) + falseNegative(count));  % Recall
    specificity(count) = trueNegative(count) / (trueNegative(count) + falsePositive(count));  % 
    precision(count) = truePositive(count) / (truePositive(count) + falsePositive(count));  % precision
    count = count + 1;
    
%     keyboard
    
%     break
    fprintf('Frame: %s\n', imName)
end

save('Examples\CamSeqManipulation\CamSeq01_Results.mat', 'truePositive', 'falsePositive', 'trueNegative', 'falseNegative', 'sensitivity', 'specificity')
% save('Examples\CamSeqManipulation\CamSeq01_EdgeResults.mat', 'truePositive', 'falsePositive', 'trueNegative', 'falseNegative', 'sensitivity', 'specificity')

% Accuracy:
accuracy = (sum(truePositive) + sum(trueNegative))/ (sum(truePositive) + sum(trueNegative) + sum(falsePositive) + sum(falseNegative));
