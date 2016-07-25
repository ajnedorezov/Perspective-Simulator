%% Load the image
im = imread('External\CamSeq01\0016E5_07959.png');
sampleRegionI = 580:650; %->
sampleRegionJ = 360:560; %v

% vid1 = VideoReader('External\seq05VD\0005VD.avi');
% for n = 1:9
%     for m = 1:30
%         im = readFrame(vid1);
%     end
% end
% sampleRegionI = 290:320; %->
% sampleRegionJ = 260:380; %v

figure,
imshow(im); hold on,
plot(sampleRegionJ([1 1 end end 1]), sampleRegionI([1 end end 1 1]), 'r')

intensityRange = 35;
% imsize = size(imresize(readFrame(vid),1/2));
imsize = size(im);
binaryIm = zeros(imsize(1), imsize(2), 3);
%%
% if true
if false
    myIPM = load('Examples\CamSeqManipulation\myIPM_smaller.mat');
    myIPM = myIPM.myIPM;    
else
    myIPM = IPM_vert(imsize(1:2),...
        'cameraZ', 7,...
        'theta', eps, ...0*pi/180,...atan(1/(2*1609.34)),...
        'stepSize', [.125 .25],...
        'xRange', [-50 50],...
        'yRange', [0 350]);
end

%
hf = figure(1);

% Create the initial snake coordinates
imsize = size(im);
    
%% Road surface removal
for n = 1:3
    channel = im(:,:,n);
    roadRegion = channel(sampleRegionI, sampleRegionJ);
    avgPixelInt = mean(roadRegion(:));
    binaryIm(:,:,n) = channel < (avgPixelInt-intensityRange) | channel > (avgPixelInt+intensityRange);
end
ind = sum(binaryIm,3)==0;

%% Perform the IPM transformation
% Transform the image
for n = 1:3
    channel = binaryIm(:,:,n);
    channel(ind) = 0;
    newVidFrame(:,:,n) = myIPM.performTransformation(double(channel));

    channel = im(:,:,n);
    rgbIPM(:,:,n) = myIPM.performTransformation(double(channel));
end

%%
figure, imshow(rot90(uint8(rgbIPM),2))
figure, imagesc(rot90(newVidFrame,2))

% save('Examples\CamSeqManipulation\myIPM_smaller.mat', 'myIPM')


%%
indIm = reshape(1:prod(imsize(1:2)), imsize(1:2));
indices = myIPM.performTransformation(indIm);
% % save('Examples\CamSeqManipulation\myIPM_smaller_nearest.mat', 'indices')
save('Examples\CamSeqManipulation\myIPM_nearest.mat', 'indices')

for n = 1:3
    channel = im(:,:,n);
    newIm(:,:,n) = channel(indices);
end

figure, imshow(rot90(uint8(newIm),2))