% Script to make images for Section 3.2 of thesis
% Comparing VP tracker to gt labels

%% Load the video
vid = VideoReader('RainVideo-Downsampled.avi');
timeRange = [47 57]; % frames 0 to 300
gtData = load('Thesis Images\Chapter 3\Data\RainVideo-Downsampled\VPLocations_0to300.mat');

%% The VP Tracker Algorithm plotting results every 0.5 sec (i.e. 15 frames)
imsize = [vid.Width vid.Height];
myVPTracker = VPTracker(1000, imsize(1:2));

while vid.CurrentTime <= timeRange(1)
    im = readFrame(vid);
end
hf = figure(1);
ax = gca(hf);

vpTrackerResults = [];
frame = 1;
while vid.CurrentTime <= timeRange(2)
% for frame = max(timeRange(1)*vid.FrameRate, 1):timeRange(2)*vid.FrameRate
%     im = read(vid, frame);
    fprintf('Frame: %d\n', frame)
    myVPTracker.Update(im, [0 0]);
    
    vpTrackerResults(frame,:) = myVPTracker.prior.mean;
    
    cla(ax);
    imshow(im, 'Parent', ax); hold on,
    
    myVPTracker.PlotResults(ax, 2)
    
    im = readFrame(vid);
    frame = frame + 1;
    drawnow
end

%% Compute the error between MCT & GT
locDelta = (vpTrackerResults - gtData.gtVanishingPoint);
error = hypot(locDelta(:,1), locDelta(:,2));

stdMagnitude = hypot(gtData.pixelDeviation(:,1), gtData.pixelDeviation(:,2));

save('Thesis Images\Chapter 3\Data\RainVideo-Downsampled\ErrorData.mat', 'locDelta', 'vpTrackerResults', 'error', 'stdMagnitude');

%% Create a plot of the error between ground truth and tracker estimate
% figure
% plot(error, 'b')
% title('Vanishing Point Estimate Error')
% ylabel('Error (pixels)')
% xlabel('Frame #')
% 
% 
% saveas(gcf, 'Thesis Images\Chapter 3\figure_3_7-MCTTrackerVsGroundTruth', 'png');

%%
figure
ax(1) = subplot(311);
plot(locDelta(:,1)), hold on, plot(gtData.pixelDeviation(:,1), 'r:')
title('Vanishing Point Estimate Error')
ylabel('|X Error|')
legend('Error', 'GT STD')

ax(2) = subplot(312);
plot(locDelta(:,2)), hold on, plot(gtData.pixelDeviation(:,2), 'r:')
ylabel('|Y Error|')
legend('Error', 'GT STD')

ax(3) = subplot(313);
plot(error)
ylabel('Error (pixels)')
xlabel('Frame #')

linkaxes(ax, 'x')

saveas(gcf, 'Thesis Images\Chapter 3\figure_3_12-MCTTrackerVsGroundTruthRain', 'png');