function Fig_5_13_through_5_15
    %% Generate the labyrinth
%     I3 = maze(5, 5, 'c', 1);   % figure 5-1
%     I1 = maze(5, 5, 'c', 3);   % figure 5-7
%     I2 = maze(5, 5, 'c', 5);   % figure 5-8
%     I = maze(5, 5, 'c', 4);   % figure 5-12
%     I = maze(5, 5, 'c', 15);   % figure 5-16, hfs2
%     I = maze(5, 5, 'c', 19);   % figure 5-17, hfs3
    I = maze(5, 5, 'c', 34);   % figure 5-17, hfs3
    
    
    [~, hfS4] = performAlgo(I, true, true);

    [~, hfS2] = performAlgo(I, true, false);
%     saveas(hfS2, 'Thesis Images\Chapter 5\figure_5_14-CounterClockwise', 'png');
    
    [~, hfS3] = performAlgo(I, false, false);
%     saveas(hfS3, 'Thesis Images\Chapter 5\figure_5_15-ReversedSlope_CounterClockwise', 'png');

    [~, hfS1] = performAlgo(I, false, true);
%     saveas(hfS1, 'Thesis Images\Chapter 5\figure_5_13-ReversedSlope', 'png');
    
    
end

function [hfGradient, hfSolution] = performAlgo(I, towardsGoal, clockwise)
    %% Create the edge map
    sigma = 2;
    f = double(I > 0.5);

    se = strel('ball',50,50);
    f = 50-imdilate(1-f,se);

    imsize = size(I);

    x1 = 115;
    y1 = 65;
    x2 = 465;
    y2 = 340;

    %% Display the figure
    % hf = figure;
    % hIm = imshow(f,[]);
    % hold on
    % title('Energy function with initial spline')    


    %% Compute the GVF of the edge map f
    tic, [px,py] = GVF(f, 0.2, 80); toc

    %% Apply GVF modifications

    % Apply a clockwise/counterclockwise rotation around the edges
    LeftRight = [zeros(size(f,1),1) diff(f, [], 2)];
    UpDown = [zeros(1,size(f,2)); diff(f, [], 1)];
    maxEdge = 1;
    
%     clockwise = true;
    % clockwise = false;
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
%     towardsGoal = true;
    % towardsGoal = false;
    [cc,rr] = meshgrid(1:imsize(2), 1:imsize(1));
    if towardsGoal
        dx = x2 - cc;
        dy = y2 - rr;
    else
        dx = x1 - cc;
        dy = y1 - rr;
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

    ind = I < 0.5;
    px(ind) = fx2(ind);
    py(ind) = fy2(ind);

    % Make the magnitude of all vectors equal
    magGVF = hypot(px,py) + 1e-10;
    px = px./magGVF;
    py = py./magGVF;

    hfGradient = figure;
    stepSize =  1;% 5;% 2;% 10;%
    [qx,qy] = meshgrid(1:stepSize:imsize(1), 1:stepSize:imsize(2));
    ind = sub2ind(imsize, qx, qy);
    quiver(qy,qx,px(ind),py(ind)); set(gca, 'ydir', 'reverse') %set(gca, 'ydir', 'normal','xdir','reverse')
    title('Gradients')
    axis([240 270 155 185])

    %% Initialize the snake
    t = linspace(0,1, 150)';
    x_s = x1 + t.*(x2-x1);
    y_s = y1 + t.*(y2-y1);

    % % Draw the initial snake
    % hold on,
    % plot(x_s, y_s, 'bo');
    % % saveas(gcf, 'Thesis Images\Chapter 5\figure_5_1-Initialized Snake', 'png');

    % Perform the snake algorithm
    hfSolution = figure;
    imshow(I,[])
    hold on,
    hSpline = plot(x_s,y_s,'bo');

    alpha = 0.05;% 0.4;%0.5; 
    beta = 0.0;%0.5;
    gamma = 1;
    kappa = 0.96;
    A = imfilter(eye(length(t)), [beta -alpha-4*beta 2*alpha+6*beta -alpha-4*beta beta], 'same', 'conv', 'circular');

    % Fix the start & endpoints 
    A(1,:) = 0; A(2,:) = 0; A(2,1:3) = [1 -2 1];
    A(end,:) = 0; A(end-1,:) = 0; A(end-1,(end-2):(end-0)) = [1 -2 1];

    % Compute the inverse of A by LU decompositions since it is a
    % pentadiagonal banded matrix
    [L,U] = lu(A + gamma*eye(size(A)));
    invA = inv(U) * inv(L);

    % Iteratively solve the Euler equations for x & y
    for n = 1:2000
        newx = gamma*x_s + kappa*interp2(px, x_s, y_s, '*linear', 0);
        newy = gamma*y_s + kappa*interp2(py, x_s, y_s, '*linear', 0);

        x_s = invA*newx;
        y_s = invA*newy;

        % Redistribute the points along the curve
        x_s([1 end]) = [x1 x2];
        y_s([1 end]) = [y1 y2];   
        dStep = cumsum(hypot([0; diff(x_s)],[0; diff(y_s)]));
        newStep = linspace(rand/max(dStep),max(dStep),length(dStep))';

        x_s = interp1(dStep,x_s,newStep);
        y_s = interp1(dStep,y_s,newStep);

        set(hSpline, 'XData', x_s, 'YData', y_s, 'Marker', 'o');
        drawnow

    end
end