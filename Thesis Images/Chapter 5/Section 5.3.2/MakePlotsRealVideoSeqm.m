%% Make the plots for section 5.3.1

% Select the video
caseNum = 4;

startingFigNum = 30;
switch(caseNum)
    case 1
        % CamSeq01 - w/o GT
        name = 'CamSeq01';
        vTimes = [1 30 60 90];
        sceneData = imread('External\CamSeq01\0016E5_07959.png');
        vid = VideoReader('Thesis Images\Chapter 5\Section 5.3.2\CamSeqMain.avi');
        rangeData = load('Thesis Images\Chapter 5\Section 5.3.2\CamSeqMain_Results.mat');
    case 2
        % CamSeq01 - w GT
        name = 'CamSeq01wGT';
        vTimes = [1 30 60 90];
        sceneData = imread('External\CamSeq01\0016E5_07959.png');
        vid = VideoReader('Thesis Images\Chapter 5\Section 5.3.2\CamSeqMainUsingGT.avi');
        rangeData = load('Thesis Images\Chapter 5\Section 5.3.2\CamSeqMainUsingGT_Results.mat');
    case 3
        % seq05VD - w/o GT
        name = 'seq05VD';
        vTimes = [1 30 60 90];
        svid = VideoReader('External\seq05VD\0005VD.avi');
        sceneData = read(svid,1);
        vid = VideoReader('Thesis Images\Chapter 5\Section 5.3.2\seq05VD.avi');
        rangeData = load('Thesis Images\Chapter 5\Section 5.3.2\seq05VD_Results.mat');
    case 4
        % seq05VD - w GT
        name = 'seq05VDwGT';
        vTimes = [1 30 60 90];
        svid = VideoReader('External\seq05VD\0005VD.avi');
        sceneData = read(svid,1);
        vid = VideoReader('Thesis Images\Chapter 5\Section 5.3.2\seq05VDUsingGT.avi');
        rangeData = load('Thesis Images\Chapter 5\Section 5.3.2\seq05VDUsingGT_Results.mat');
end
    
% %% Create the results plot
% hf = figure('Position', [25 100 1200 400]);
% ax(1) = subplot(1,2,1);
% imshow(sceneData);
% title(ax(1), 'Scenerio View')
% 
% ax(2) = subplot(1,2,2);
% t = 1:length(rangeData.downRangeToObstacleOnPath);
% solutionPath = rangeData.downRangeToObstacleOnPath;
% solutionPath(isinf(solutionPath)) = 300;
% directPath = rangeData.closestDownRangeToObstacle;
% directPath (isinf(directPath )) = 300;
% plot(ax(2), t, solutionPath , 'b-o', t, directPath, 'r:x', 'Linewidth', 1.5)
% legend('Solution Path', 'Direct Path', 'Location', 'best')
% title(ax(2), 'Planning Performance')
% ylabel(ax(2), 'Down Range (ft)')
% xlabel(ax(2), 'Frame #')
% axis(ax(2), [t([1 end]) 0 310])
% 
% 
% saveas(hf, sprintf('Thesis Images\\Chapter 5\\figure_5_%d-Case_%s_Results', startingFigNum + 2*(caseNum-1), name), 'png');
% 
% valids = solutionPath ~= 300 | directPath ~= 300;
% fprintf('lessThan: %d\n', sum((solutionPath(valids)-directPath(valids))<5))
% fprintf('equalTo: %d\n', sum(abs(solutionPath(valids)-directPath(valids))<5))
% fprintf('greaterThan: %d\n', sum((solutionPath(valids)-directPath(valids))>5))
% fprintf('#Opp: %d\n', sum(valids))

%% Create the sequence plot 
% Override the subplot function with one that will make it tighter
addpath(genpath('subtight'))
msubplot = @(m,n,p,extra) subtightplot (m, n, p, [0.01 0.05], [0.01 0.01], [0.01 0.01]);

hf = figure('position', [5 150 1283 737]);
for n = 1:length(vTimes)
    ax = msubplot(1, 4, n);
    frame = read(vid, vTimes(n));
    imshow(frame(:,80:410,:) , 'Parent', ax);
    title(sprintf('                Frame: %d', vTimes(n)));
end

saveas(hf, sprintf('Thesis Images\\Chapter 5\\figure_5_%d-Case_%s_Sequence', startingFigNum + 1 + 2*(caseNum-1), name), 'png');