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
myVid = VideoWriter('Road Surface Substraction - Planning.avi');
open(myVid);

% sampleRegionI = (370:2:400)/2; %->
% sampleRegionJ = (360:2:560)/2; %v
sampleRegionI = 370:400; %->
sampleRegionJ = 360:560; %v
intensityRange = 35;
% imsize = size(imresize(readFrame(vid),1/2));
imsize = size(readFrame(vid));
binaryIm = zeros(imsize(1), imsize(2), 3);

%%
if true
    myIPM = load('Examples\Road Surface Substraction\myIPM.mat');
    myIPM = myIPM.myIPM;    
else
    myIPM = IPM_vert(imsize(1:2),...
        'cameraZ', 7,...
        'theta', eps,...0.003125*pi/180,...atan(1/(2*1609.34)),...
        'stepSize', [.125 .25],...
        'xRange', [-50 50],...
        'yRange', [0 350]);
end

%%
hf = figure(1);

% Create the initial snake coordinates
imsize = [1401 801];
x_s = linspace(0,imsize(1),10);
y_s = linspace(imsize(2)/2,imsize(2)/2,10);

% Upsample & create a spline
steps = 0:(length(x_s)-1);
newSteps = 0:0.05:(length(x_s)-1);
pp = spline(steps,[x_s' y_s']', newSteps);
x_s = pp(1,:)';
y_s = pp(2,:)';

while hasFrame(vid)
    if ~ishandle(hf)
        break
    end
    
    vidFrame = readFrame(vid);
%     vidFrame = imresize(readFrame(vid),1/2);
%     fprintf('Current Time: %f\n', vid.CurrentTime);
%     if ~(vid.CurrentTime > 10 && vid.CurrentTime < 20)
%         continue
%     end
%     newVidFrame = vidFrame;
%     vidFrame = im2double(vidFrame);
    for n = 1:3
        channel = vidFrame(:,:,n);
        roadRegion = channel(sampleRegionI, sampleRegionJ);
        avgPixelInt = mean(roadRegion(:));
        binaryIm(:,:,n) = channel < (avgPixelInt-intensityRange) | channel > (avgPixelInt+intensityRange);
    end
    ind = sum(binaryIm,3)==0;
    for n = 1:3
        channel = binaryIm(:,:,n);
        channel(ind) = 0;
        newVidFrame(:,:,n) = myIPM.performTransformation(double(channel));
    end
%     grayIm = rgb2gray(newVidFrame);
%     newVidFrame = binaryIm;
    
    clf(figure(1)), 
    subplot(211), imshow(vidFrame)
    title(sprintf('Current Time: %f', vid.CurrentTime))
    ylabel('Original')
%     subplot(212), imshow(255*double(binaryIm))% imshow(newVidFrame)% 
%     subplot(212), imagesc(grayIm)
    ax = subplot(212); imagesc(newVidFrame)
    ylabel('IPM Road Surface Substracted')
    
    set(ax,'yDir','normal','xdir','reverse')
    
    
    %% Do the maze path following algorithm stuff
    ipmIm = newVidFrame;
    
%                 ipmVP = [15 300];

%     % Get the current vanishing point estimate
%     origvpx = (ipmVP(1)-self.aVars.IPM.xRange(1))*size(ipmIm,1)/diff(self.aVars.IPM.xRange);
%     origvpy = ipmVP(2)*size(ipmIm,2)/diff(self.aVars.IPM.yRange);
% 
%     % Limit vp to point in image
%     m = (origvpx-size(ipmIm,1)/2)/origvpy;
%     vpy = size(ipmIm,2);
%     vpx = vpy*m + size(ipmIm,1)/2;

    vpy = size(ipmIm,2)/2;
    vpx = size(ipmIm,1);

    % Create an edge map for the path planning
%     Im = ismember(newLabels, find(obstacles));
    Im = rgb2gray(ipmIm) > 0;
    imsize = size(Im);

    smArea = 50;
    se = strel('ball', smArea, smArea);
    smoothedIm = smArea-imdilate(1-Im, se);

    % Create a curve towards the vanishing point
    [xx,yy] = meshgrid(1:imsize(2), 1:imsize(1));
    dx = vpy - xx;
    dy = vpx - yy;
    newMag =  sqrt(dx.*dx + dy.*dy);
    newMag = 1 - newMag / max(newMag(:));
%                 newMag = newMag / max(newMag(:));

%                 % Figure out what direction the edges are going
%                 LeftRight = [zeros(size(smoothedIm,1),1) diff(smoothedIm, [], 2)];
%                 UpDown = [zeros(1,size(smoothedIm,2)); diff(smoothedIm, [], 1)];
% %                 h = fspecial('gaussian', [5 5]);
% %                 LeftRight = imfilter(LeftRight, h);
% %                 UpDown = imfilter(UpDown, h);

    % Create the GVF
    mu = 0.2;
    smoothedIm = padarray(smoothedIm, [1 1], 'symmetric', 'both');
    [fx,fy] = gradient(-smoothedIm);

    u = fx;
    v = fy;
    newMag = padarray(newMag, [1 1], 'symmetric', 'both');
    [fx2, fy2] = gradient(newMag);
    fx2 = fx2(2:end-1,2:end-1);
    fy2 = fy2(2:end-1,2:end-1);

    gradMag = u.*u + v.*v;

    for n = 1:160
        u = padarray(u(2:end-1, 2:end-1), [1 1], 'symmetric', 'both');
        v = padarray(v(2:end-1, 2:end-1), [1 1], 'symmetric', 'both');
        u = u + mu*4*del2(u) - gradMag.*(u-fx);
        v = v + mu*4*del2(v) - gradMag.*(v-fy);
    end
    u = u(2:end-1,2:end-1);
    v = v(2:end-1,2:end-1);

    maxEdge = max(max(hypot(u,v))); %0.25
    edgeX = maxEdge*fx2./(hypot(fx2,fy2) + eps);
    edgeY = maxEdge*fy2./(hypot(fx2,fy2) + eps);

%                 clockwise = true;
% %                 clockwise = false;
%                 if clockwise 
%                     v(LeftRight < 0) = -maxEdge/2;    u(LeftRight < 0) = 0;
%                     v(LeftRight > 0) = maxEdge/2;     u(LeftRight < 0) = 0;
%                     v(UpDown < 0) = 0;              u(UpDown < 0) = maxEdge/2;
%                     v(UpDown > 0) = 0;              u(UpDown > 0) = -maxEdge/2;
%                 else
%                     v(LeftRight < 0) = maxEdge/2;    u(LeftRight < 0) = 0;
%                     v(LeftRight > 0) = -maxEdge/2;     u(LeftRight < 0) = 0;
%                     v(UpDown < 0) = 0;              u(UpDown < 0) = -maxEdge/2;
%                     v(UpDown > 0) = 0;              u(UpDown > 0) = maxEdge/2;
%                 end   

%                 se = strel('ball', 13, 13);
%                 ind = imdilate(double(Im > 0.5), se);
%                 ind = ~logical(ind-min(ind(:)));
% 
%                 u(ind) = edgeX(ind);
%                 v(ind) = edgeY(ind);
% %                 u(Im < 0.5) = edgeX(Im < 0.5);
% %                 v(Im < 0.5) = edgeY(Im < 0.5);

%                 ind = hypot(u,v) < 1e-4;
%                 u(ind) = edgeX(ind);
%                 v(ind) = edgeY(ind);

    u = u + edgeX/2;
    v = v + edgeY/2;

%                 ind = hypot(u,v) < 1e-2;
%                 u(ind) = u(ind) + edgeX(ind)/10;
%                 v(ind) = v(ind) + edgeY(ind)/10;

    magGVF = hypot(u,v) + 1e-10;
    fx = u./magGVF;
    fy = v./magGVF;

%                 fx = u;
%                 fy = v;

    % Create the components of the Euler equation
    % [Tension, rigidity, stepsize, energy portion]
    alpha = 0.7;% 0.1;% 0.4;%0.5; 
    beta = 0.5;%0.0;%0.5;
    gamma = 1;
    kappa = 1;
    A = imfilter(eye(length(newSteps)), [beta -alpha-4*beta 2*alpha+6*beta -alpha-4*beta beta], 'same', 'conv', 'circular');

    % Compute the inverse of A by LU decompositions since it is a
    % pentadiagonal banded matrix
    [L,U] = lu(A + gamma*eye(size(A)));
    invA = inv(U) * inv(L);

    % Iteratively solve the Euler equations for x & y
%                 tempFig = figure(999);
%                 tempax = gca(tempFig);
%                 cla(tempax);
%                 imagesc(smoothedIm), colormap gray, hold on,
%                 hSpline = plot(tempax, y_s, x_s, 'b-o');

    for n = 1:400
        newx = gamma*x_s + kappa*interp2(fy, x_s, y_s, '*linear', 0);
        newy = gamma*y_s + kappa*interp2(fx, x_s, y_s, '*linear', 0);

        x_s = invA*newx;
        y_s = invA*newy;

        % Redistribute the points along the curve
        x_s([1 end]) = [0 vpx];
        y_s([1 end]) = [imsize(2)/2 vpy];
        dStep = cumsum(hypot([0; diff(x_s)],[0; diff(y_s)]));
        newStep = linspace(rand/max(dStep),max(dStep),length(dStep))';
%                     dStep = cumsum(hypot(diff(x_s),diff(y_s)));
%                     newStep = linspace(rand/max(dStep),max(dStep),length(dStep))';
        x_s = interp1(dStep,x_s,newStep);
        y_s = interp1(dStep,y_s,newStep);

%                     set(hSpline, 'XData', y_s, 'YData', x_s, 'Marker', 'o');
%                     drawnow
    end
    
    hold on
    plot(ax, y_s, x_s)
    
    
    writeVideo(myVid, getframe(figure(1)));

    
end
% close(myVid)