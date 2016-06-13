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
myVid = VideoWriter('scrap.avi');
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
myVPTracker = VPTracker(1000, imsize(1:2));

%%
if true
% if false
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
ptInFrontOfCar = [400 330];

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
    
    %% Grab the next video frame
%     for n = 1:4
        vidFrame = readFrame(vid);
%     end
%     vidFrame = imresize(readFrame(vid),1/2);
%     fprintf('Current Time: %f\n', vid.CurrentTime);
%     if ~(vid.CurrentTime > 10 && vid.CurrentTime < 20)
%         continue
%     end
%     newVidFrame = vidFrame;
%     vidFrame = im2double(vidFrame);


    %% Update the VP Tracker
    control = [0 0];%[delT self.RoadParams.laneWidth*sin(t/25) + 3*self.RoadParams.laneWidth/2];
    myVPTracker.Update(vidFrame, control);

    %% Road surface removal
    for n = 1:3
        channel = vidFrame(:,:,n);
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
        
        channel = vidFrame(:,:,n);
        rgbIPM(:,:,n) = myIPM.performTransformation(double(channel));
    end
%     grayIm = rgb2gray(newVidFrame);
%     newVidFrame = binaryIm;

    % Transform the vanishing point
%     ipmVP = myIPM.transformSinglePoint(max(myVPTracker.prior.mean(2), myIPM.rHorizon), myVPTracker.prior.mean(1));
    ipmVP = myIPM.transformSinglePoint(myVPTracker.prior.mean(2), myVPTracker.prior.mean(1));
    origvpx = (ipmVP(1)-myIPM.xRange(1))*size(newVidFrame,2)/diff(myIPM.xRange);
    origvpy = (ipmVP(2)-myIPM.yRange(1))*size(newVidFrame,1)/diff(myIPM.yRange);

    % Limit vp to point in image
    m = (origvpx-size(newVidFrame,2)/2)/origvpy;
    vpy = size(newVidFrame,1);
    vpx = vpy*m + size(newVidFrame,2)/2;
    
    % Plot the current results
    clf(figure(1)), 
    subplot(121), imshow(vidFrame), hold on, plot(myVPTracker.prior.mean(1), myVPTracker.prior.mean(2), 'ms', 'markersize', 6, 'linewidth', 3)
    title(sprintf('Current Time: %f', vid.CurrentTime))
    ylabel('Original')
%     subplot(212), imshow(255*double(binaryIm))% imshow(newVidFrame)% 
%     subplot(212), imagesc(grayIm)
%     ax = subplot(122); imagesc(newVidFrame), hold on, plot(vpx, vpy, 'ms', 'markersize', 6, 'linewidth', 3)
%    
%     ylabel('IPM Road Surface Substracted')
%     
%     set(ax,'yDir','normal','xdir','reverse')
    
    %% Detect obstacles by checking if its a horizontal streak
    % Perform K-means to segment the image into different
    % regions
    
    
%     useKmeans = true;
    useKmeans = false;
    
    if useKmeans
        numCusters = 3;
        grayIm = rgb2gray(newVidFrame);
%         grayIm = double(rgb2gray(uint8(rgbIPM)));
        
        grayIm(isnan(grayIm)) = -1;
        [IDX, centers] = kmeans(grayIm(:), numCusters);
        IDX = reshape(IDX, size(grayIm));
        IDX(IDX == IDX(1,1)) = -1;     % Grab the first pixel since this is always black because the IPM creates a cone and this pixel is outside of the interpolated data

        % Figure out the color of the roadway
%         roadPixel = grayIm(round(size(grayIm,2)/2) + (-5:5), 80+(0:5));
        roadPixel = grayIm(ptInFrontOfCar(1) + (-20:20), ptInFrontOfCar(2)+(-10:30)); %ptInFrontOfCar = [400 330];
        roadPixelRange = median(roadPixel(:)) + 50*[-1 1];%0.05*[-1 1];

        % Break each cluster into individual unique clusters,
        % concurrently, decide if the cluster is roadway
        newLabels = zeros(size(IDX));
        offset = 0;
        roadLabels = [];
        for n = 1:length(centers)
            ind = IDX == n;
            tempLabels = bwlabeln(ind) + offset;
            newLabels(ind) = tempLabels(ind & newLabels==0);
            offset = max(newLabels(:));
            % NOTE this conditional logic needs to change so it'll be more adaptive
            % to roadway light fluctuations (i.e. use the color of the road right
            % in front of the camera, this means we need to maintain some distance
            % between vehicles)
        %     if centers(n) > .33 && centers(n) < .4
            if centers(n) > roadPixelRange(1) && centers(n) < roadPixelRange(2)
                % These belong to the road
                roadLabels = [roadLabels; unique(tempLabels)];
            end
        end
    else
        grayIm = sum(newVidFrame,3) > 0;
        newLabels = bwlabeln(grayIm);
    end

    % Get the region properties for the segments
    stats = regionprops(newLabels, 'BoundingBox', 'Extent', 'Orientation');

    % Decide if the cluster is streak like and something that
    % should be avoided
    obstacles = false(length(stats),1);
    for n = 1:length(stats)
%         obstacles(n) = ~ismember(n, roadLabels) && stats(n).BoundingBox(4) > 100 && stats(n).BoundingBox(4) > stats(n).BoundingBox(3) && stats(n).BoundingBox(3) > 30;
        obstacles(n) = stats(n).BoundingBox(4) > 100 && stats(n).BoundingBox(4) > stats(n).BoundingBox(3) && stats(n).BoundingBox(3) > 30;
    end
%     obstacles = true(length(stats),1);
    
    % Plot the obstacles around the IPM image
%     ipmSize = size(newLabels);
%     isObstacle = uint8(zeros([ipmSize(1:2) 3]));
    isObstacle(:,:,1) = uint8(ismember(newLabels, find(obstacles)));
%     for n = 1:3
%         channel = rgbIPM(:,:,n);
%         channel(~isObstacle) = 0;
%         rgbIPM(:,:,n) = channel;
%     end
    
    
    ax = subplot(122); 
    tIm = imoverlay(uint8(rgbIPM), isObstacle,[1 0 0]);
    imshow(tIm, 'Parent', ax)
%     imagesc(uint8(rgbIPM)),
    hold on, plot(vpx, vpy, 'ms', 'markersize', 6, 'linewidth', 3)
    ylabel('IPM Road Surface Substracted')
    set(ax,'yDir','normal','xdir','reverse')
    
%     for n = find(obstacles)'
%         if ~obstacles(n)
%             continue
%         end
% %                     x = (stats(n).BoundingBox(1) + [0 0 stats(n).BoundingBox([3 3]) 0])*sy;
% %                     y = (stats(n).BoundingBox(2) + [0 stats(n).BoundingBox([4 4]) 0 0])*sx + self.aVars.IPM.xRange(1);
% % %                     newPts = self.aVars.IPM.transformSinglePoint(y, x);
% % %                     plot(ax, x, y, 'y', 'linewidth', 1.5);
% %                     plot(ax, y, x, 'y', 'linewidth', 1.5);
% %                     plot(ax, stats(n).BoundingBox(1) + [0 0 stats(n).BoundingBox([3 3]) 0], stats(n).BoundingBox(2) + [0 stats(n).BoundingBox([4 4]) 0 0], 'y', 'linewidth', 1.5);
%         plot(ax, stats(n).BoundingBox(1) + [0 0 stats(n).BoundingBox([3 3]) 0], stats(n).BoundingBox(2) + [0 stats(n).BoundingBox([4 4]) 0 0], 'y', 'linewidth', 1.5);
%     end
    
    
    %% Do the maze path following algorithm stuff
%     ipmIm = newVidFrame;
% 
%     % Create an edge map for the path planning
% %     Im = ismember(newLabels, find(obstacles));
%     Im = rgb2gray(ipmIm) > 0;
%     imsize = size(Im);
% 
%     smArea = 50;
%     se = strel('ball', smArea, smArea);
%     smoothedIm = smArea-imdilate(1-Im, se);
% 
%     % Create a curve towards the vanishing point
%     [xx,yy] = meshgrid(1:imsize(2), 1:imsize(1));
%     dx = vpy - xx;
%     dy = vpx - yy;
%     newMag =  sqrt(dx.*dx + dy.*dy);
%     newMag = 1 - newMag / max(newMag(:));
% %                 newMag = newMag / max(newMag(:));
% 
% %                 % Figure out what direction the edges are going
% %                 LeftRight = [zeros(size(smoothedIm,1),1) diff(smoothedIm, [], 2)];
% %                 UpDown = [zeros(1,size(smoothedIm,2)); diff(smoothedIm, [], 1)];
% % %                 h = fspecial('gaussian', [5 5]);
% % %                 LeftRight = imfilter(LeftRight, h);
% % %                 UpDown = imfilter(UpDown, h);
% 
%     % Create the GVF
%     mu = 0.2;
%     smoothedIm = padarray(smoothedIm, [1 1], 'symmetric', 'both');
%     [fx,fy] = gradient(-smoothedIm);
% 
%     u = fx;
%     v = fy;
%     newMag = padarray(newMag, [1 1], 'symmetric', 'both');
%     [fx2, fy2] = gradient(newMag);
%     fx2 = fx2(2:end-1,2:end-1);
%     fy2 = fy2(2:end-1,2:end-1);
% 
%     gradMag = u.*u + v.*v;
% 
%     for n = 1:80%160
%         u = padarray(u(2:end-1, 2:end-1), [1 1], 'symmetric', 'both');
%         v = padarray(v(2:end-1, 2:end-1), [1 1], 'symmetric', 'both');
%         u = u + mu*4*del2(u) - gradMag.*(u-fx);
%         v = v + mu*4*del2(v) - gradMag.*(v-fy);
%     end
%     u = u(2:end-1,2:end-1);
%     v = v(2:end-1,2:end-1);
% 
%     maxEdge = max(max(hypot(u,v))); %0.25
%     edgeX = maxEdge*fx2./(hypot(fx2,fy2) + eps);
%     edgeY = maxEdge*fy2./(hypot(fx2,fy2) + eps);
% 
% %                 clockwise = true;
% % %                 clockwise = false;
% %                 if clockwise 
% %                     v(LeftRight < 0) = -maxEdge/2;    u(LeftRight < 0) = 0;
% %                     v(LeftRight > 0) = maxEdge/2;     u(LeftRight < 0) = 0;
% %                     v(UpDown < 0) = 0;              u(UpDown < 0) = maxEdge/2;
% %                     v(UpDown > 0) = 0;              u(UpDown > 0) = -maxEdge/2;
% %                 else
% %                     v(LeftRight < 0) = maxEdge/2;    u(LeftRight < 0) = 0;
% %                     v(LeftRight > 0) = -maxEdge/2;     u(LeftRight < 0) = 0;
% %                     v(UpDown < 0) = 0;              u(UpDown < 0) = -maxEdge/2;
% %                     v(UpDown > 0) = 0;              u(UpDown > 0) = maxEdge/2;
% %                 end   
% 
% %                 se = strel('ball', 13, 13);
% %                 ind = imdilate(double(Im > 0.5), se);
% %                 ind = ~logical(ind-min(ind(:)));
% % 
% %                 u(ind) = edgeX(ind);
% %                 v(ind) = edgeY(ind);
% % %                 u(Im < 0.5) = edgeX(Im < 0.5);
% % %                 v(Im < 0.5) = edgeY(Im < 0.5);
% 
% %                 ind = hypot(u,v) < 1e-4;
% %                 u(ind) = edgeX(ind);
% %                 v(ind) = edgeY(ind);
% 
%     u = u + edgeX/2;
%     v = v + edgeY/2;
% 
% %                 ind = hypot(u,v) < 1e-2;
% %                 u(ind) = u(ind) + edgeX(ind)/10;
% %                 v(ind) = v(ind) + edgeY(ind)/10;
% 
%     magGVF = hypot(u,v) + 1e-10;
%     fx = u./magGVF;
%     fy = v./magGVF;
% 
% %                 fx = u;
% %                 fy = v;
% 
%     % Create the components of the Euler equation
%     % [Tension, rigidity, stepsize, energy portion]
%     alpha = 0.7;% 0.1;% 0.4;%0.5; 
%     beta = 0.5;%0.0;%0.5;
%     gamma = 1;
%     kappa = 1;
%     A = imfilter(eye(length(newSteps)), [beta -alpha-4*beta 2*alpha+6*beta -alpha-4*beta beta], 'same', 'conv', 'circular');
% 
%     % Compute the inverse of A by LU decompositions since it is a
%     % pentadiagonal banded matrix
%     [L,U] = lu(A + gamma*eye(size(A)));
%     invA = inv(U) * inv(L);
% 
%     % Iteratively solve the Euler equations for x & y
% %                 tempFig = figure(999);
% %                 tempax = gca(tempFig);
% %                 cla(tempax);
% %                 imagesc(smoothedIm), colormap gray, hold on,
% %                 hSpline = plot(tempax, y_s, x_s, 'b-o');
% 
%     for n = 1:100%400
%         newx = gamma*x_s + kappa*interp2(fy, x_s, y_s, '*linear', 0);
%         newy = gamma*y_s + kappa*interp2(fx, x_s, y_s, '*linear', 0);
% 
%         x_s = invA*newx;
%         y_s = invA*newy;
% 
%         % Redistribute the points along the curve
%         x_s([1 end]) = [0 vpx];
%         y_s([1 end]) = [imsize(2)/2 vpy];
%         dStep = cumsum(hypot([0; diff(x_s)],[0; diff(y_s)]));
%         newStep = linspace(rand/max(dStep),max(dStep),length(dStep))';
% %                     dStep = cumsum(hypot(diff(x_s),diff(y_s)));
% %                     newStep = linspace(rand/max(dStep),max(dStep),length(dStep))';
%         x_s = interp1(dStep,x_s,newStep);
%         y_s = interp1(dStep,y_s,newStep);
% 
% %                     set(hSpline, 'XData', y_s, 'YData', x_s, 'Marker', 'o');
% %                     drawnow
%     end
%     
%     hold on
%     plot(ax, y_s, x_s)
%     axis equal
    
    try
        writeVideo(myVid, getframe(figure(1)));
        
    end
end
close(myVid)