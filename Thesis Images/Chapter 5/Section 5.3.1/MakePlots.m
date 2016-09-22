%% Make the plots for section 5.3.1

% Select the video
caseNum = 3;

startingFigNum = 22;
switch(caseNum)
    case 1
        vTimes = [1 4 12 19];
    case 2
        vTimes = [1 4 10 19];
    case 3
        vTimes = [1 4 11 21];
    case 4
        vTimes = [1 34 61 85];  
end
    
%% Create the results plot
hf = figure('Position', [25 100 1000 700]);
ax(1) = subplot(2,2,1);
sceneData = load(sprintf('Thesis Images\\Chapter 5\\Section 5.3.1\\Case %d\\sceneView.mat', caseNum));
imshow(sceneData.sceneView);
title(ax(1), 'Scenerio View')

histData = load(sprintf('Thesis Images\\Chapter 5\\Section 5.3.1\\Case %d\\historyLogging.mat', caseNum));
t = ((1:length(histData.posHistory)) - 1)*1/3;

% ax(2) = subplot(2,2,2);
% plot(ax(2), t, histData.posHistory(:,2), 'b', t, histData.objectPosHistory(:,[2:3:size(histData.objectPosHistory,2)]), 'r', [0 t(end)], ')
% legend('Obstacle
% title(ax(2), 'Cross Range')
% set(ax(2), 'YTick', [1 3 5]*3/2, 'YTickLabel', {'Lane 3', 'Lane 2', 'Lane 1'}, 'ylim', [0 9], 'yDir', 'reverse')
ax(2) = subplot(2,2,2);
h1 = plot(ax(2), histData.posHistory(:,1), histData.posHistory(:,2), 'b', histData.objectPosHistory(:,[1:3:size(histData.objectPosHistory,2)]), histData.objectPosHistory(:,[2:3:size(histData.objectPosHistory,2)]), 'r', 'Linewidth', 2.25);
hold on,
h2 = plot(ax(2), [-10 10000], [0 0; 2 2; 4 4; 6 6]*3/2, 'k--', 'xLimInclude', 'off', 'Linewidth', 1.5);
legend([h1(1:2); h2(1)], 'Autonomous Vehicle', 'Obstacle Vehicles', 'Lane Markings', 'Location', 'best')
ylabel('Cross Range (m)')
xlabel('Down Range (m)')
title(ax(2), 'Position Plot')
set(ax(2), 'YTick', [1 3 5]*3/2, 'YTickLabel', {'Lane 3', 'Lane 2', 'Lane 1'}, 'ylim', [-0.1 9.1], 'yDir', 'reverse')

ax(3) = subplot(2,2,3);
rangeData = load(sprintf('Thesis Images\\Chapter 5\\Section 5.3.1\\Case %d\\rangeInfo.mat', caseNum));
solutionPath = rangeData.downRangeToObstacleOnPath;
solutionPath(isinf(solutionPath)) = 300;
directPath = rangeData.closestDownRangeToObstacle;
directPath (isinf(directPath )) = 300;
plot(ax(3), t, solutionPath , 'b-o', t, directPath, 'r:x', 'Linewidth', 1.5)
legend('Solution Path', 'Direct Path', 'Location', 'best')
title(ax(3), 'Planning Performance')
ylabel(ax(3), 'Down Range (ft)')
xlabel(ax(3), 'Time (s)')
axis(ax(3), [t([1 end]) 0 310])

ax(4) = subplot(2,2,4);
plot(ax(4),t, histData.yawHistory*180/pi, 'b', t, histData.yawCmdHistory*180/pi, 'r:', 'Linewidth', 3)
legend('Heading', 'Commanded Heading', 'Location', 'best')
xlabel(ax(4), 'Time (s)') 
ylabel(ax(4), 'Heading (deg)');
title(ax(4), 'Heading')

% saveas(hf, sprintf('Thesis Images\\Chapter 5\\figure_5_%d-Case%dResults', startingFigNum + 2*(caseNum-1), caseNum), 'png');

deltaThresh = 3;
fprintf('# opportunities: %d\n', sum(directPath~=300 | solutionPath~=300));
fprintf('# greater than: %d\n', sum((solutionPath - directPath) > deltaThresh));
fprintf('# less than: %d\n', sum((solutionPath - directPath) < -deltaThresh));
fprintf('# equal to: %d\n', sum(abs(solutionPath - directPath) < deltaThresh & directPath~=300));


% %% Create the sequence plot 
% % Override the subplot function with one that will make it tighter
% addpath(genpath('subtight'))
% msubplot = @(m,n,p,extra) subtightplot (m, n, p, [0.01 0.01], [0.01 0.01], [0.01 0.01]);
% 
% vid = VideoReader(sprintf('Thesis Images\\Chapter 5\\Section 5.3.1\\Case %d\\Video.avi', caseNum));
% 
% hf = figure('position', [5 150 1283 737]);
% for n = 1:length(vTimes)
%     ax = msubplot(1, 4, n);
%     frame = read(vid, vTimes(n));
%     imshow(frame, 'Parent', ax);
%     title(sprintf('Time: %02.2f sec', (vTimes(n)-1)/vid.FrameRate));
% end
% 
% saveas(hf, sprintf('Thesis Images\\Chapter 5\\figure_5_%d-Case%dSequence', startingFigNum + 1 + 2*(caseNum-1), caseNum), 'png');