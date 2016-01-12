Ic = imread('BaseIm.png');
Ic = rgb2gray(Ic);

Ic = double(Ic)/255;

myIPM = IPM(size(Ic),...
    'cameraZ', 4.4,...
    'theta', atan(1/(0.5*1609.34)),...
    'stepSize', [.125 .25],...
    'yRange', [0 300]);

tic
Iw = myIPM.performTransformation(Ic);
% Iw = myIPM.performTransformation(Ic, 2);
toc

Iw = rot90(Iw);

RI = imref2d(size(Iw), myIPM.yRange, myIPM.xRange);

figure(1);
imshow(Iw, RI); colormap(gray);
axis image;
ax = gca;
title(ax, 'Bertozzi and Broggi''s Mapping Results');
xlabel(ax, 'Down Range (ft)');
ylabel(ax, 'Cross Range (ft)');
