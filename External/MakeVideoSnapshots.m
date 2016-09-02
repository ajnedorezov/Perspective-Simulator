%% Select the video
% vid = VideoReader('Videos - 20160726\FollowWaypoints.avi'); vTimes = [35 90 145 184]; sbs = [2,2];
% vid = VideoReader('Videos - 20160726\ObstacleEnRoutToWaypoint.avi'); vTimes = [15 30 86 111]; sbs = [2,2];
% vid = VideoReader('Videos - 20160726\MovingObstacle.avi'); vTimes = [10 22 74 111]; sbs = [2,2];
% vid = VideoReader('Videos - 20160726\ObstacleOnWaypoint.avi'); vTimes = [20 65 79 146 177 211]; sbs = [2,3];
% vid = VideoReader('Videos - 20160726\SandbarObstacle.avi'); vTimes = [39 128 215 309.5]; sbs = [2,2];
% vid = VideoReader('Videos - 20160726\BankInWayOfWaypoint.avi'); vTimes = [38 171 260 438]; sbs = [2,2];
vid = VideoReader('Videos - 20160726\NarrowAndShallowCrossing.avi'); vTimes = [11 65 151 234 470 540]; sbs = [2,3];

%% Override the subplot function with one that will make it tighter
addpath(genpath('subtight'))
subplot = @(m,n,p,extra) subtightplot (m, n, p, [0.01 0.01], [0.01 0.01], [0.01 0.01]);

%% Place the video frames into subplots
% scrsz = get(0,'ScreenSize'); % left, bottom, width, height
hf = figure;%('position',[1 scrsz(4)/100 scrsz(3)/0.5/2 scrsz(4)]);
for n = 1:length(vTimes)
    ax = subplot(sbs(1), sbs(2), n);
%     subpos = get(ax, 'Position');
%     set(ax, 'Position', subpos.*[1 1 1.2 1.2]);
    
    frame = read(vid, vTimes(n)/0.5);
    
    imshow(frame(:, 1:448, :), 'Parent', ax);
end

