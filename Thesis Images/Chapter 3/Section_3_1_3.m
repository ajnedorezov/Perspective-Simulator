% Script to make images for Section 3.1.3 of thesis

%% Load the video
vid = VideoReader('Downsampled To Work Video.avi');

%% The VP Tracker Algorithm plotting results every 0.5 sec (i.e. 15 frames)
imsize = [vid.Width vid.Height];
myVPTracker = VPTracker(1000, imsize(1:2));

hf = figure;
firstFrame = true;
count = 1;
ax = subplot(3,3,count);
while vid.CurrentTime < 1.5
    im = readFrame(vid);
    cla(ax);
    imshow(im, 'Parent', ax);
    myVPTracker.Update(im, [0 0]);
    
    if mod(vid.CurrentTime, 5/vid.FrameRate) == 0 || firstFrame
        firstFrame = false;
        hold on
        myVPTracker.PlotResults(ax, 2)
        title(sprintf('Elapsed Time: %f',vid.CurrentTime - 1/vid.FrameRate))  % First frame time is 1 frame rate ahead, subtract it
%         saveas(hf, ['Thesis Images\Chapter 3\figure_3_6-Frame' sprintf('%d.png',vid.CurrentTime*vid.FrameRate)], 'png');
        count = count + 1;
        if count <= 9
            ax = subplot(3,3,count);
        end
    end
end

saveas(hf, 'Thesis Images\Chapter 3\figure_3_6-VPConvergence', 'png');