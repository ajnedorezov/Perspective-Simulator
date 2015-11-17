clc
clear all
close all

% Highway driving, changing lanes
a =  mmreader('C:\Program Files (x86)\MATLAB\R2011a Student\toolbox\vision\visiondemos\viplanedeparture.avi');


vidFrames = read(a);
hFig1 = figure;
ax1 = axes('Parent', hFig1);
hFig2 = figure;
ax2 = axes('Parent', hFig2);
movegui(hFig1, 'northwest')
movegui(hFig2, 'southwest')

mov = struct([]);
edges = struct([]);
for k = 1:a.NumberOfFrames
    fprintf('Processing frame %d of %d\n', k, a.NumberOfFrames)
    mov(k).cdata = vidFrames(:,:,:,k);
    mov(k).colormap = [];
    
    % Perform edge detection
%     edges(k).cdata = edge(rgb2gray(vidFrames(:,:,:,k)), 'canny'); % More lines
    edges(k).cdata = edge(rgb2gray(vidFrames(:,:,:,k)), 'sobel'); % Less lines
    edges(k).colomap = [];    
    
    % Estimate the lines 
    [H,T,R] = hough(edges(k).cdata);
    P  = houghpeaks(H,20);%,'threshold',ceil(0.3*max(H(:))));
    lines = houghlines(edges(k).cdata, T, R, P); %,'FillGap',5,'MinLength',7 );
    xy = zeros(2,3,length(lines));
    for l = 1:length(lines)
        xy(:,:,l) = [[lines(l).point1; lines(l).point2] nan(2,1)];
    end
    
    if length(lines) > 1
        % Estimate the line intersections
        slopes = squeeze(diff(xy,1,1))';
        A = slopes(:,[2 1]);
        b = sum(A.*squeeze(xy(1,1:2,:))',2);
        [m n] = size(A);
        if (n < m)
            A(:,end+1:end+m) = eye(m);
            [m n] = size(A);
        end

        t = nchoosek(1:n,m);
        nv = nchoosek(n,m);
        vert = nan(n,nv);
        for j=1:nv
            y = zeros(n,1);
            x = A(:,t(j,:))\b;
            y(t(j,:)) = x;
%             if all(y(1:2) >= 0 & (y(1:2) ~= inf & y(1:2) ~= -inf))
                vert(:,j) = y;% = [vert y];
%             end
        end
    else
         vert = nan(2,1);
         fprintf('\tOnly 1 line found this frame\n')
    end
    
    try
        imshow(mov(k).cdata, 'Parent', ax1)
        imshow(edges(k).cdata, 'Parent', ax2)
        hold(ax2, 'on')
        plot(ax2, xy(1,:), xy(2,:), 'c')
        plot(ax2, vert(1,:), vert(2,:), 'rx', 'MarkerSize', 10, 'LineWidth', 2)
        title(ax2, sprintf('Frame %d of %d', k, a.NumberOfFrames))
        drawnow
    catch %#ok<CTCH>
        % One of the figures was closed stop break out of the loop
        delete(findall(0, 'Type', 'figure'))
        break
    end
end
