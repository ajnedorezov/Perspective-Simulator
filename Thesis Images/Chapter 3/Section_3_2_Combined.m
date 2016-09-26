case1 = load('Thesis Images\Chapter 3\Data\Downsampled To Work\ErrorData.mat');
case2 = load('Thesis Images\Chapter 3\Data\Downsampled To Work 2\ErrorData.mat');
case3 = load('Thesis Images\Chapter 3\Data\seq05VD\ErrorData.mat');
case4 = load('Thesis Images\Chapter 3\Data\RainVideo-Downsampled\ErrorData.mat');

%% Combined Error
hf = figure;

h = plot(1:length(case1.error), case1.error, 1:length(case2.error), case2.error, 1:length(case3.error), case3.error, 1:length(case4.error), case4.error);
ylabel('Error (pixels)')
xlabel('Frame #')

set(h, 'Linewidth', 3)

legend('Straight Highway', 'Curved Highway', 'Fauqueur', 'Rainy')

% saveas(hf, 'Thesis Images\Chapter 3\figure_3_7-ErrorPlot', 'png')

%% X & Y Error
figure
ax(1) = subplot(211);
h=plot(1:length(case1.error), (case1.locDelta(:,1)), 1:length(case2.error), (case2.locDelta(:,1)), 1:length(case3.error), (case3.locDelta(:,1)), 1:length(case4.error), (case4.locDelta(:,1)));
title('Vanishing Point Estimate Error')
ylabel('X Error')
legend('Error', 'GT STD')
legend('Straight Highway', 'Curved Highway', 'Fauqueur', 'Rainy')
set(h, 'Linewidth', 3)

ax(2) = subplot(212);
h=plot(1:length(case1.error), (case1.locDelta(:,2)), 1:length(case2.error), (case2.locDelta(:,2)), 1:length(case3.error), (case3.locDelta(:,2)), 1:length(case4.error), (case4.locDelta(:,2)));
ylabel('Y Error')
legend('Error', 'GT STD')
legend('Straight Highway', 'Curved Highway', 'Fauqueur', 'Rainy')
set(h, 'Linewidth', 3)

linkaxes(ax, 'x')

saveas(gcf, 'Thesis Images\Chapter 3\figure_3_8-XYErrorPlot', 'png');



% figure, 
% h = plot(1:length(case1.error), case1.error/960, 1:length(case2.error), case2.error/960, 1:length(case3.error), case3.error/480, 1:length(case4.error), case4.error/960);