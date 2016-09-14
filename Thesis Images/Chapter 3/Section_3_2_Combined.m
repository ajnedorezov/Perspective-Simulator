case1 = load('Thesis Images\Chapter 3\Data\Downsampled To Work\ErrorData.mat');
case2 = load('Thesis Images\Chapter 3\Data\Downsampled To Work 2\ErrorData.mat');
case3 = load('Thesis Images\Chapter 3\Data\seq05VD\ErrorData.mat');
case4 = load('Thesis Images\Chapter 3\Data\RainVideo-Downsampled\ErrorData.mat');



figure

h = plot(1:length(case1.error), case1.error, 1:length(case2.error), case2.error, 1:length(case3.error), case3.error, 1:length(case4.error), case4.error);
ylabel('Error (pixels)')
xlabel('Frame #')

set(h, 'Linewidth', 3)

legend('Straight Highway', 'Curved Highway', 'Fauqueur', 'Rainy')
