function test_ComputeLines
    clc
    clear all
    close all

    % Read in an image
    im = imread('Test Images\pic002.jpg');
%     im = imread('Test Images\pic003.png');
    movegui(figure, 'northwest')
    imshow(im)   
    
    % Perform edge detection
    greyim = rgb2gray(im);
    imedge = edge(greyim, 'sobel');
%     imedge = edge(greyim, 'canny');
    movegui(figure, 'southwest')
    imshow(imedge);
    
    % Find the x & y derivatives
    xdir = imfilter(int8(greyim), [1 -1]);
    ydir = imfilter(int8(greyim), [1 -1]');
%     movegui(figure, 'north')
%     subplot(121)
%     imshow(xdir)
%     subplot(122)
%     imshow(ydir)
    
    % Draw the direction of the x & y derivatives for each of the edges.
    [xx, yy] = meshgrid(1:size(greyim,2), 1:size(greyim,1));
    figure(2)
    hold on
    quiver(xx(imedge), yy(imedge), xdir(imedge), ydir(imedge))
    
    % Break the image into a much coarser grid and average the x & y
    % derivates within that grid space;
    gridsize = 6;
    [sx, sy] = size(greyim);
    xstartind = round(linspace(1, sy, gridsize));
    ystartind = round(linspace(1, sx, gridsize));
    
    % Compute the average gradient for each grid space
    sector = struct;
    for xind = 1:length(xstartind)-1
        for yind = 1:length(ystartind)-1
            sector(xind, yind).xmean = ...
                mean( xdir( imedge( ystartind(xind):ystartind(xind+1),...
                                    xstartind(yind):xstartind(yind+1))));
            sector(xind, yind).ymean = ...
                mean( ydir (imedge( ystartind(xind):ystartind(xind+1),...
                                    xstartind(yind):xstartind(yind+1)))); 
            % If x/y mean is nan then there are no derivatives in this
            % region. Set to zero.
            if isnan(sector(xind, yind).xmean) || isnan(sector(xind, yind).ymean)
                sector(xind, yind).xmean = 0;
                sector(xind, yind).ymean = 0;
            end
            sector(xind, yind).xpos = (xstartind(xind)+xstartind(xind+1))/2;
            sector(xind, yind).ypos = (ystartind(yind)+ystartind(yind+1))/2;
            
            t = sector(xind, yind);
            quiver(t.xpos, t.ypos, t.xmean, t.ymean, 'color', 'r');
        end
    end
    
    % Create the A and b matrix
    A = zeros(numel(sector), 2);
    b = zeros(numel(sector), 1);
    count = 1;
    for xind = 1:length(xstartind)-1
        for yind = 1: length(ystartind)-1
            t = sector(xind, yind);
%             A(count, :) = [-t.ymean t.xmean];
%             b(count) = t.xmean*A(count, :)*[t.xpos t.ypos]';
            A(count, :) = [-t.ymean/t.xmean 1];
            b(count) = A(count, :)*[t.xpos t.ypos]';
            if isnan(A(count,1))
                A(count, 1) = 0;
                b(count) = 0;
            end
            count = count + 1;
        end
    end
    
    % Compute the intercept points
    pts = findVert(A, b);
    if isempty(pts)
        disp('No intersections found')
    else
        plot(pts(1,:), pts(2,:), 'cx')
        plot(mean(pts(1,:)), mean(pts(2,:)), 'mo')
    end
    

end

function pts = findVert(A, b)
    pts = [];
    
    [numConstraints, numVariables] = size(A);
    
    % If num constraints > num variables
    if numConstraints > numVariables
        A = [A eye(numConstraints)];
        [numConstraints, numVariables] = size(A);
%         if size(A,1) ~= size(A,2)
%             error('Unable to make A square')
%         end
    end

    ind = nchoosek(1:numVariables, numConstraints);
    % Ignore combinations that don't use the first two variables
    temp = (ind == 1 | ind == 2);
    ind(~(temp(:,1) & temp(:,2)), :) = [];

    numCombinations = size(ind, 1);%nchoosek(n,m);
    for k = 1:numCombinations
        y = zeros(numVariables,1);
        % Solve the eqution using the current set of variables
        x = A(:,ind(k,:))\b;
        if all(isinf(x) | isnan(x))
            % Ingore parallel lines
            continue
        end
        
        % Store the values into the proper indicies
        y(ind(k,:)) = x;
        pts = [pts y];
    end
end


