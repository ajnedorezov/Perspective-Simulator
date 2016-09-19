function makeSection5_1_3_Images% Script to make images for Section 5.1.3 of thesis

%     % Figure 5-18
%     im = imread('Thesis Images\Chapter 4\BaseImageSection_4_0.png');
%     sampleRegionI = 370:400; %->
%     sampleRegionJ = 360:560; %v
%     myIPM = load('Examples\Road Surface Substraction\myIPM.mat');
%     myIPM = myIPM.myIPM; 
%     imsize = size(im);
% 
%     [rgbIPM, isObstacle, ~] =  classifyObstacles(im, sampleRegionI, sampleRegionJ, myIPM);
% 
%     hf = figure;
%     subplot(121)
%     tIm = imoverlay(uint8(rgbIPM), isObstacle, [1 0 0]);
%     imshow(rot90(tIm,2))
%     title('Obstalces in IPM Frame')
%     subplot(122)
%     imshow(uint8(255*rot90(isObstacle,2)))
%     title('Binary Mapping of Obstacles')
%     hold on
%     imsize = size(isObstacle);
%     plot(imsize(2)/2*[1 1], [1 imsize(1)-160], 'r', 'linewidth', 3);
% 
%     saveas(hf, 'Thesis Images\Chapter 5\figure_5_18-IPMObstacleMapping', 'png');
    
    %% Figure 5-19
%     im = load('Thesis Images\Chapter 5\Section 5.1.3\BaseImageSection_5_1_3.mat');
%     im = uint8(im.rgb);
%     savedIPM = load('@IPM\myIPM.mat');
% 	myIPM = savedIPM.myIPM;
%     sampleRegionI = 400:450; %->
%     sampleRegionJ = 270:370; %v
%     
%     [rgbIPM, isObstacle, isExpandedObstacle] =  classifyObstacles(im, sampleRegionI, sampleRegionJ, myIPM);
%     
%     figure,
%     ax(1) = subplot(121);
%     ax(2) = subplot(122);
%     tIm = imoverlay(uint8(rgbIPM), isObstacle, [1 0 0]);
%     imshow(rot90(tIm,2), 'Parent', ax(1))
%     title(ax(1), 'Original Obstacles')
%     tIm = imoverlay(uint8(rgbIPM), isExpandedObstacle, [1 0 0]);
%     imshow(rot90(tIm,2), 'Parent', ax(2))
%     title(ax(2), 'Obstacles After Expansion Buffer')
%     saveas(gcf, 'Thesis Images\Chapter 5\figure_5_19-IPMwithExpandedObstacles', 'png');
    
    %% Figure 5-20
% %     im = imread('Thesis Images\Chapter 4\BaseImageSection_4_0.png');
% %     sampleRegionI = 370:400; %->
% %     sampleRegionJ = 360:560; %v
% %     myIPM = load('Examples\Road Surface Substraction\myIPM.mat');
% %     myIPM = myIPM.myIPM; 
% %     [~, ~, isExpandedObstacle] =  classifyObstacles(im, sampleRegionI, sampleRegionJ, myIPM);
%     
% %     [hfResults(1), hSnakeClockwise] = performSnake(isExpandedObstacle, true, true, 160);
% %     [hfResults(2), hSnakeCounterClockwise] = performSnake(isExpandedObstacle, true, false, 160);
% %     set(hSnakeClockwise, 'LineWidth', 3, 'Color', 'm', 'Marker', 'none', 'LineStyle', '-')
% %     set(hSnakeCounterClockwise, 'LineWidth', 3, 'Color', 'm', 'Marker', 'none', 'LineStyle', '-')
% %     title(gca(hfResults(1)), 'Clockwise')
% %     title(gca(hfResults(2)), 'Counter-clockwise')
% 
%     uiopen('Thesis Images\Chapter 5\Section 5.1.3\fig_5-20_clockwise.fig',1)
%     uiopen('Thesis Images\Chapter 5\Section 5.1.3\fig_5-20_counterclockwise.fig',1)
% %     hfResults = findall(0, 'type', 'figure');
%     hfResults = flipud(findall(0, 'type', 'figure'));
%     
%     addpath(genpath('subtight'))
%     subplot = @(m,n,p,extra) subtightplot (m, n, p, [0.01 0.01], [0.01 0.05], [0.01 0.01]);
%     
%     hf = figure;
%     ax(1) = subplot(1,2,1);
%     ax(2) = subplot(1,2,2);
%     for i = 1:2
%         h = get(hfResults(i),'Children');
%         newh = copyobj(h,hf);
%         for j = 1:length(newh)
% %             posnewh = get(newh(j),'Position');
%             possub  = get(ax(i),'Position');
%             set(newh(j),'Position',...
%                 possub)
%         end
%         delete(ax(i));
%     end    
% %     delete(hSnakeClockwise);
% %     delete(hSnakeCounterClockwise);
%     keyboard
%     saveas(hf, 'Thesis Images\Chapter 5\figure_5_20-RotationalEffect', 'png');
end


function [rgbIPM, isObstacle, isExpandedObstacle] =  classifyObstacles(im, sampleRegionI, sampleRegionJ, myIPM)
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

    for n = 1:3
        channel = binaryIm(:,:,n);
        channel(ind) = 0;
        newVidFrame(:,:,n) = myIPM.performTransformation(double(channel));

        channel = im(:,:,n);
        rgbIPM(:,:,n) = myIPM.performTransformation(double(channel));
    end

    grayIm = sum(newVidFrame,3) > 0;
    newLabels = bwlabeln(grayIm);

    % Get the region properties for the segments
    stats = regionprops(newLabels, 'BoundingBox', 'Extent', 'Orientation');

    % Decide if the cluster is streak like and something that
    % should be avoided
    obstacles = false(length(stats),1);
    for n = 1:length(stats)
        obstacles(n) = stats(n).BoundingBox(4) > 100 && stats(n).BoundingBox(4) > stats(n).BoundingBox(3) && stats(n).BoundingBox(3) > 30;
    end

    isObstacle(:,:,1) = uint8(ismember(newLabels, find(obstacles)));

    isExpandedObstacle(:,:,1) = imdilate(ismember(newLabels, find(obstacles)), true(1,35));
end

function [hfSolution, hSpline] = performSnake(I, towardsGoal, clockwise, y1)

    imsize = size(I);

    x1 = imsize(2)/2;
    if ~exist('y1', 'var')
        y1 = 1;
    end
    x2 = imsize(2)/2;
    y2 = imsize(1);
    
    
    cx = x1;
    cy = y1;
    vpx = x2;
    vpy = y2;

    %% Apply GVF modifications
    isObstacle = I;
    % Smooth the image and convert to an edge map
    smArea = 50;
    se = strel('ball', smArea, smArea);
    GI = imdilate(double(isObstacle), se)-smArea;
    f = smArea-GI;

    % Compute the GVF of the edge map f
    [px,py] = GVF(f, 0.2, 40);

%                 % Make the magnitude of all vectors equal
%                 magGVF = hypot(px,py) + 1e-10;
%                 px = px./magGVF;
%                 py = py./magGVF;

    % Make the obstacle have vectors towards the vanishing point
    imsize = size(px);

    % Apply a clockwise/counterclockwise rotation around the edges
    LeftRight = [zeros(size(f,1),1) diff(f, [], 2)];
    UpDown = [zeros(1,size(f,2)); diff(f, [], 1)];
    maxEdge = 1;

    if clockwise 
        py(LeftRight < 0) = -maxEdge/2;    px(LeftRight < 0) = 0;
        py(LeftRight > 0) = maxEdge/2;     px(LeftRight > 0) = 0;
        py(UpDown < 0) = 0;
        py(UpDown > 0) = 0;
        px(UpDown < 0) = maxEdge/2;
        px(UpDown > 0) = -maxEdge/2;
    else
        py(LeftRight < 0) = maxEdge/2;    px(LeftRight < 0) = 0;
        py(LeftRight > 0) = -maxEdge/2;     px(LeftRight > 0) = 0;
        px(UpDown < 0) = -maxEdge/2;
        px(UpDown > 0) = maxEdge/2;
    end    

    % Apply a slope towards the goal point
    [cc,rr] = meshgrid(1:imsize(2), 1:imsize(1));
    if towardsGoal
        dx = vpx - cc;
        dy = vpy - rr;
    else
        dx = vpx - cc;
        dy = vpy - rr;
    end
    newMag =  sqrt(dx.*dx + dy.*dy) + eps;
    newMag = 1 - newMag./max(max(newMag));
    [fx2,fy2] = gradient(newMag);
    mag = hypot(fx2,fy2) + eps;
    fx2 = fx2./mag;
    fy2 = fy2./mag;

    pmag = max(max(hypot(px,py)));
    px = px/pmag;
    py = py/pmag;

    px(isObstacle) = fx2(isObstacle);
    py(isObstacle) = fy2(isObstacle);

    % Make the magnitude of all vectors equal
    magGVF = hypot(px,py) + 1e-10;
    px = px./magGVF;
    py = py./magGVF;

%     hfGradient = figure;
%     stepSize = 5;%  1;% 2;% 10;%
%     [qx,qy] = meshgrid(1:stepSize:imsize(1), 1:stepSize:imsize(2));
%     ind = sub2ind(imsize, qx, qy);
%     quiver(qy,qx,px(ind),py(ind)); set(gca, 'ydir', 'reverse') %set(gca, 'ydir', 'normal','xdir','reverse')
%     title('Gradients')
% %     axis([240 270 155 185])

    %% Initialize the snake
    snakeTime = linspace(0,1,100)';
    snakeX = cx + snakeTime.*(vpx-cx); % vpx.*t + (1-t).*cx;
    snakeY = cy + snakeTime.*(vpy-cy);
    
    hfSolution = figure;
    imshow(I,[])
    hold on,
    hSpline = plot(snakeX,snakeY,'bo');
    set(gca(hfSolution),'yDir','normal','xdir','reverse')
    
    for n = 1:100
        snakeX(end) = vpx;
        snakeY(end) = vpy;
        snakeX(1) = cx;
        snakeY(1) = cy;
        
        [snakeX,snakeY] = snakedeform(snakeX,snakeY,1,0.25,0.25,5,px,py,1);
    
        
        
        set(hSpline, 'XData', snakeX, 'YData', snakeY);
        drawnow
    end
    
end


