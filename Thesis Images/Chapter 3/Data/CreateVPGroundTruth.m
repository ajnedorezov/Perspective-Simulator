% Script to create the ground truth position of the vanishing point

%% Load the video
vid = VideoReader('Downsampled To Work Video.avi');
timeRange = [0 10];

%% Collect VP data
% For each frame, display it ask the user where the vanishing point is, and
% log it for averaging latter

hf = figure;
ax = gca;

while vid.CurrentTime <= timeRange(1)
    im = readFrame(vid);
end

startFrameTime = vid.CurrentTime;
frame = 1;
while vid.CurrentTime <= timeRange(2)
    cla(ax);
    imshow(im, 'Parent', ax);
    title(sprintf('ElapsedTime: %f', vid.CurrentTime-1/vid.FrameRate));
    drawnow
    gtVanishingPoint(frame, :) = ginput(1);
    im = readFrame(vid);
    frame = frame + 1;
end

%% Save the data off
save('Thesis Images\Chapter 3\Data\VPLocations_0to300_SampleSet3', 'gtVanishingPoint');


return

%% Cluster the different data sets into 1 final version
numSets = 2;
xData = [];
yData = [];
for n = 1:numSets
    data = load(['Thesis Images\Chapter 3\Data\' sprintf('VPLocations_0to300_SampleSet%d', n)]);
    xData(:, end+1) = data.gtVanishingPoint(:,1);
    yData(:, end+1) = data.gtVanishingPoint(:,2);
end

% Compute the new means & stds
gtVanishingPoint(:,1) = mean(xData,2);
gtVanishingPoint(:,2) = mean(yData,2);

pixelDeviation(:,1) = std(xData,[],2);
pixelDeviation(:,2) = std(yData,[],2);

save('Thesis Images\Chapter 3\Data\VPLocations_0to300', 'gtVanishingPoint', 'pixelDeviation');