% Script to make images for Section 4.2.2 of thesis

%% Load the image
im = imread('Thesis Images\Chapter 4\SimulatedImageSection_4_2_2.png');

figure, 
imshow(im)
title('Simulated Perspective Image')

saveas(gcf, 'Thesis Images\Chapter 4\figure_4_6-SimulatedScene', 'png');


%% Display the overhead image
overheadIm = imread('Thesis Images\Chapter 4\OverheadImageSection_4_2_2.png');

% Parameters used in simulation to get the scene
%                 posvec(1) = 75;
%                 posvec(3) = 300;
%                 targvec(1) = posvec(1) + 1;
%                 targvec(2) = posvec(2) + 0;

figure, 
imshow(overheadIm)
title('Simulated Overhead Image')

saveas(gcf, 'Thesis Images\Chapter 4\figure_4_7-BeforeRoadRemoval', 'png');

%% Apply the Tuohy Road Substraction Algorithm
savedIPM = load('@IPM\myIPM.mat');
myIPM = savedIPM.myIPM;

for n = 1:3
    channel = im(:,:,n);
    rgbIPM(:,:,n) = myIPM.performTransformation(double(channel));
end

% rgbIPM = imread('Thesis Images\Chapter 4\IPMImageSection_4_2_2.png');

figure, 
imshow(rot90(uint8(rgbIPM),2))
title('IPM Transformed Image')

saveas(gcf, 'Thesis Images\Chapter 4\figure_4_8-IPMTransformedSimulated', 'png');


%% Apply IPM to figure 4-3
im = imread('Thesis Images\Chapter 4\BaseImageSection_4_2_1.png');

clear myIPM rgbIPM
myIPM = load('Examples\Road Surface Substraction\myIPM.mat');
myIPM = myIPM.myIPM;

for n = 1:3
    channel = im(:,:,n);
    rgbIPM(:,:,n) = myIPM.performTransformation(double(channel));
end

figure, 
imshow(rot90(uint8(rgbIPM),2))
title('IPM Transformed Image')

saveas(gcf, 'Thesis Images\Chapter 4\figure_4_9-IPMTransformedRealWorld', 'png');


