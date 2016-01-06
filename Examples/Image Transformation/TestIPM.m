Ic = imread('BaseIm.png');
Ic = rgb2gray(Ic);

Ic = double(Ic)/255;

myIPM = IPM(size(Ic),...
    'cameraZ', 4.4,...
    'theta', atan(1/(0.5*1609.34)));

tic
Iw = myIPM.performTransformation(Ic);
toc

Iw = rot90(Iw);
figure;
imagesc(Iw,[0 1]); colormap(gray);
axis image;
title('Bertozzi and Broggi''s Mapping Results');
xlabel('Column');
ylabel('Row');
