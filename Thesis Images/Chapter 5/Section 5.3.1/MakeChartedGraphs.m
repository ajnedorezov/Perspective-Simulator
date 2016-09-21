%% Select the video
vid = VideoReader('Thesis Images\Chapter 5\Section 5.3.1\PathIntersectingObstacles-Case1_v2.avi'); vTimes = [1 4 11 18]; sbs = [1,4];
% vid = VideoReader('Thesis Images\Chapter 5\Section 5.3.1\PathIntersectingObstacles-Case2_v2.avi'); vTimes = [1 3 10 18]; sbs = [1,4];
% vid = VideoReader('Thesis Images\Chapter 5\Section 5.3.1\PathIntersectingObstacles-Case3_v2.avi'); vTimes = [1 5 33 51]; sbs = [1,4];
% vid = VideoReader('Thesis Images\Chapter 5\Section 5.3.1\PathIntersectingObstacles-Case4_v2.avi'); vTimes = [1 4 11 21]; sbs = [1,4];

%% Override the subplot function with one that will make it tighter
addpath(genpath('subtight'))
subplot = @(m,n,p,extra) subtightplot (m, n, p, [0.01 0.01], [0.01 0.01], [0.01 0.01]);
%% Place the video frames into subplots

% scrsz = get(0,'ScreenSize'); % left, bottom, width, height
% hf = figure;%('position',[1 scrsz(4)/100 scrsz(3)/0.5/2 scrsz(4)]);
hf = figure('position', [50         150        1283         737]);
for n = 1:length(vTimes)
    ax = subplot(sbs(1), sbs(2), n);
    
    frame = read(vid, vTimes(n));
    
%     imshow(frame(:, 1:448, :), 'Parent', ax);
    imshow(frame, 'Parent', ax);
    title(sprintf('Time: %02.2f sec', (vTimes(n)-1)/vid.FrameRate));
end

