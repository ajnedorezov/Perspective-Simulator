% Script to make images for Section 4.2.3 of thesis

%% Load the image
im = imread('Thesis Images\Chapter 4\BaseImageSection_4_2_1.png');
sampleRegionI = 370:400; %->
sampleRegionJ = 360:560; %v

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

myIPM = load('Examples\Road Surface Substraction\myIPM.mat');
myIPM = myIPM.myIPM; 

for n = 1:3
    channel = binaryIm(:,:,n);
    channel(ind) = 0;
    newVidFrame(:,:,n) = myIPM.performTransformation(double(channel));
    
    channel = im(:,:,n);
    rgbIPM(:,:,n) = myIPM.performTransformation(double(channel));
end

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

%% Disaply the new result
figure,
tIm = imoverlay(uint8(rgbIPM), allObstacles, [1 0 0]);
imshow(rot90(tIm,2))
title('IPM with Road Surface Substracted Overlay')
% set(ax,'yDir','normal','xdir','reverse')

title('IPM Transformed Image')

saveas(gcf, 'Thesis Images\Chapter 4\figure_4_10-IPMofRoadSubtractedImageAllObstacles', 'png');


%% Result with all obstacles detected
figure,
tIm = imoverlay(uint8(rgbIPM), isObstacle, [1 0 0]);
imshow(rot90(tIm,2))
title('IPM with Road Surface Substracted Overlay')
% set(ax,'yDir','normal','xdir','reverse')

title('IPM Transformed Image')

saveas(gcf, 'Thesis Images\Chapter 4\figure_4_11-IPMofRoadSubtractedImage', 'png');


