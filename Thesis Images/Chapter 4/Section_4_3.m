% Script to make images for Section 4.3 of thesis

%% Load the image
% % imName = 'External\CamSeq01\0016E5_07959';
% imName = 'External\CamSeq01\0016E5_08045';
% im = imread([imName '.png']);
% labelIm = imread([imName '_L.png']);
% sampleRegionI = 580:650; %->
% sampleRegionJ = 360:560; %v

vid1 = VideoReader('External\seq05VD\0005VD.avi');
vid2 = VideoReader('External\seq05VD\05VD_preview.mpg');
im = readFrame(vid1);
labelIm = readFrame(vid2);
sampleRegionI = 290:320; %->
sampleRegionJ = 260:380; %v

figure,
imshow(im); hold on,
plot(sampleRegionJ([1 1 end end 1]), sampleRegionI([1 end end 1 1]), 'r')

%% Apply the Tuohy Road Substraction Algorithm
intensityRange = 35;

binaryIm = [];
for n = 1:3
    channel = im(:,:,n);
    roadRegion = channel(sampleRegionI, sampleRegionJ);
    avgPixelInt = mean(roadRegion(:));
    binaryIm(:,:,n) = channel < (avgPixelInt-intensityRange) | channel > (avgPixelInt+intensityRange);
end
ind = sum(binaryIm,3)==0;

% myIPM = load('Examples\CamSeqManipulation\myIPM.mat');
myIPM = load('Examples\CamSeqManipulation\myIPM_smaller.mat');
myIPM = myIPM.myIPM; 

% nearestIPM = load('Examples\CamSeqManipulation\myIPM_nearest.mat');
nearestIPM = load('Examples\CamSeqManipulation\myIPM_smaller_nearest.mat');
for n = 1:3
    channel = binaryIm(:,:,n);
    channel(ind) = 0;
    newVidFrame(:,:,n) = myIPM.performTransformation(double(channel));
    
    channel = im(:,:,n);
    rgbIPM(:,:,n) = myIPM.performTransformation(double(channel));
    
    channel = labelIm(:,:,n);
    ipmchannel = channel(nearestIPM.indices);
    ipmchannel(nearestIPM.indices==1) = 0;
    labelIPM_color(:,:,n) = ipmchannel;
end

% r = labelIm(:,:,1);
% g = labelIm(:,:,2);
% b = labelIm(:,:,3);
% nonObstacle = r==128 & g==0 & b==192 |... %LaneMkgsDriv
%               r==128 & g==64 & b==128; %Road
% 
% labelIPM = double(nonObstacle(nearestIPM.indicies));
% figure, imagesc(rot90(labelIPM,2));

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

isObstacle(:,:,1) = uint8(ismember(newLabels, find(obstacles)));

obstacles = true(length(stats),1);
allObstacles = uint8(ismember(newLabels, find(obstacles)));

%% 
figure
imshow(uint8(rot90(labelIPM_color,2)));

%% Disaply the new result
figure,
tIm = imoverlay(uint8(rgbIPM), uint8(allObstacles), [1 0 0]);
imshow(rot90(tIm,2))
title('IPM with Road Surface Substracted Overlay')
% set(ax,'yDir','normal','xdir','reverse')

title('IPM Transformed Image')

% saveas(gcf, 'Thesis Images\Chapter 4\figure_4_10-IPMofRoadSubtractedImageAllObstacles', 'png');


%% Result with all obstacles detected
figure,
tIm = imoverlay(uint8(rgbIPM), isObstacle, [1 0 0]);
imshow(rot90(tIm,2))
title('IPM with Road Surface Substracted Overlay')
% set(ax,'yDir','normal','xdir','reverse')

%% Result with all obstacles detected
figure,
tIm = imoverlay(uint8(labelIPM_color), isObstacle, [1 0 0]);
imshow(rot90(tIm,2))
title('IPM with Road Surface Substracted Overlay')
% set(ax,'yDir','normal','xdir','reverse')
title('IPM Transformed Image')

% saveas(gcf, 'Thesis Images\Chapter 4\figure_4_11-IPMofRoadSubtractedImage', 'png');


