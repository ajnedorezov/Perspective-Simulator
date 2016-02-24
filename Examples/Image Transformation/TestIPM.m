% Ic = imread('BaseIm.png');
% Ic = rgb2gray(Ic);
% 
% Ic = double(Ic)/255;

vid = VideoReader('Downsampled To Work Video.avi');
Ic = readFrame(vid);
Ic = imresize(Ic, 1/2);
Ic = rgb2gray(Ic);
Ic = double(Ic)/255;

figure(3)
imshow(Ic)
title('Original Image')

myIPM = IPM(size(Ic),...
    'cameraZ', 7,...
    'theta', eps,...0.003125*pi/180,...atan(1/(2*1609.34)),...
    'stepSize', [.125 .25],...
    'xRange', [-50 50],...
    'yRange', [0 350]);

tic
% Iw = myIPM.performTransformation(Ic);
Iw = myIPM.performTransformation(Ic, 2);
toc

% Iw = rot90(Iw,2);

%%
RI = imref2d(size(Iw), myIPM.xRange, myIPM.yRange);

figure(1);
imshow(Iw, RI); colormap(gray);
% axis image;
ax = gca;
title(ax, 'Bertozzi and Broggi''s Mapping Results');
set(ax,'ydir', 'normal', 'xdir', 'reverse')
ylabel(ax, 'Down Range (ft)');
xlabel(ax, 'Cross Range (ft)');
% axis(ax,'square')
axis(ax,'fill')
axis(ax, [myIPM.xRange, myIPM.yRange])