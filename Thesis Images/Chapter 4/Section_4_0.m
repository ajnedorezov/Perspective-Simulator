% Script to make images for Section 4.0 of thesis

%% Load the image
im = imread('Thesis Images\Chapter 4\BaseImageSection_4_0.png');
sampleRegionI = 370:400; %->
sampleRegionJ = 360:560; %v

figure, 
imshow(im)
title('Original Image')

saveas(gcf, 'Thesis Images\Chapter 4\figure_4_1-CapturedImage', 'png');

%% Apply the Tuohy Road Substraction Algorithm
myIPM = load('Examples\Road Surface Substraction\myIPM.mat');
myIPM = myIPM.myIPM; 

for n = 1:3
    channel = im(:,:,n);
    rgbIPM(:,:,n) = myIPM.performTransformation(double(channel));
end

figure,
imshow(rot90(uint8(rgbIPM),2))
title('IPM Transformed Image')

saveas(gcf, 'Thesis Images\Chapter 4\figure_4_2-IPMTransformedImage', 'png');



