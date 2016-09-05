%% Select the video
% vid = VideoReader('Thesis Images\Chapter 5\Section 5.3.1\PathIntersectingObstacles-Case1.avi'); vTimes = [1 10 28 48]; sbs = [1,4];
vid = VideoReader('Thesis Images\Chapter 5\Section 5.3.1\PathIntersectingObstacles-Case2.avi'); vTimes = [1 10 28 48]; sbs = [1,4];
% vid = VideoReader('Thesis Images\Chapter 5\Section 5.3.1\PathIntersectingObstacles-Case3.avi'); vTimes = [1 10 88 136]; sbs = [1,4];

%% Override the subplot function with one that will make it tighter
addpath(genpath('subtight'))
subplot = @(m,n,p,extra) subtightplot (m, n, p, [0.01 0.01], [0.01 0.01], [0.01 0.01]);
%% Place the video frames into subplots

% scrsz = get(0,'ScreenSize'); % left, bottom, width, height
% hf = figure;%('position',[1 scrsz(4)/100 scrsz(3)/0.5/2 scrsz(4)]);
hf = figure('position', [139         227        1283         737]);
for n = 1:length(vTimes)
    ax = subplot(sbs(1), sbs(2), n);
    
    frame = read(vid, vTimes(n));
    
%     imshow(frame(:, 1:448, :), 'Parent', ax);
    imshow(frame, 'Parent', ax);
    title(sprintf('Time: %02.2f sec', vTimes(n)/vid.FrameRate));
end

