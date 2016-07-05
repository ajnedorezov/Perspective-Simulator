% Script to make images for Section 4.2.1 of thesis

%% Load the image
im = imread('Thesis Images\Chapter 4\BaseImageSection_4_2_1.png');
sampleRegionI = 370:400; %->
sampleRegionJ = 360:560; %v

figure, 
imshow(im)
title('Original Image Prior to Road Surface Removal')

saveas(gcf, 'Thesis Images\Chapter 4\figure_4_3-BeforeRoadRemoval', 'png');

%% Apply the Tuohy Road Substraction Algorithm
intensityRange = 35;

binaryIm = [];
for n = 1:3
    channel = im(:,:,n);
    roadRegion = channel(sampleRegionI, sampleRegionJ);
    avgPixelInt = mean(roadRegion(:));
    binaryIm(:,:,n) = channel < (avgPixelInt-intensityRange) | channel > (avgPixelInt+intensityRange);
end
ind = sum(binaryIm,3)==0;

figure,
imshow(binaryIm)
title('Touhy Road Surface Substraction')

saveas(gcf, 'Thesis Images\Chapter 4\figure_4_4-AfterRoadRemoval', 'png');





