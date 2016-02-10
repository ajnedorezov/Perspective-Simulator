% %%
% im = imread('freeway.jpg');
% 
% figure, imshow(im);
% 
% %%
% sampleRegionI = 190:220; %->
% sampleRegionJ = 130:250; %v
% intensityRange = 35;
% binaryIm = zeros(size(im));
% 
% tic
% for m = 1:100
%     for n = 1:3
%         channel = im(:,:,n);
%         roadRegion = channel(sampleRegionI, sampleRegionJ);
%         avgPixelInt = mean(roadRegion(:));
%         binaryIm(:,:,n) = channel < (avgPixelInt-intensityRange) | channel > (avgPixelInt+intensityRange);
%     end
% end
% toc
% % figure, imshow(255*double(binaryIm))
% 
% 
% % %% more optimized way - NOT!!!!!!
% % binaryIm2 = zeros(size(im));
% % tic
% % for n = 1:100
% %     roadRegion = im(sampleRegionI,sampleRegionJ,:);
% %     avgPixelInt = mean(mean(roadRegion,1),2);
% % 
% %     binaryIm2 = bsxfun(@(x,y) x < (y-intensityRange) | x > (y+intensityRange), im, avgPixelInt);
% % 
% %     % for n = 1:3
% %     %     binaryIm2(:,:,n) = im(:,:,n) < (avgPixelInt(n)-intensityRange) | im(:,:,n) > (avgPixelInt(n)+intensityRange);
% %     % end
% % end
% % toc
% % 
% % % figure, imshow(255*double(binaryIm2))

%% Test road substraction for video

vid = VideoReader('Downsampled To Work Video.avi');
% myVid = VideoWriter('Road Surface Substraction - MultiColor.avi');
% open(myVid);

sampleRegionI = 370:400; %->
sampleRegionJ = 360:560; %v
intensityRange = 35;
binaryIm = zeros(vid.Height, vid.Width, 3);

hf = figure(1);
while hasFrame(vid)
    if ~ishandle(hf)
        break
    end
    vidFrame = readFrame(vid);
    newVidFrame = vidFrame;
    for n = 1:3
        channel = vidFrame(:,:,n);
        roadRegion = channel(sampleRegionI, sampleRegionJ);
        avgPixelInt = mean(roadRegion(:));
        binaryIm(:,:,n) = channel < (avgPixelInt-intensityRange) | channel > (avgPixelInt+intensityRange);
    end
    ind = sum(binaryIm,3)==0;
    for n = 1:3
        channel = newVidFrame(:,:,n);
        channel(ind) = 0;
        newVidFrame(:,:,n) = channel;
    end
    grayIm = rgb2gray(newVidFrame);
    
    clf(figure(1)), 
    subplot(211), imshow(vidFrame)
    title(sprintf('Current Time: %f', vid.CurrentTime))
    ylabel('Original')
%     subplot(212), imshow(255*double(binaryIm))% imshow(newVidFrame)% 
    subplot(212), imagesc(grayIm)% imshow(newVidFrame)% 
    ylabel('Road Surface Substracted')
    
%     if vid.CurrentTime > 10 && vid.CurrentTime < 20
%         writeVideo(myVid, getframe(figure(1)));
%     end

    
end
% close(myVid)