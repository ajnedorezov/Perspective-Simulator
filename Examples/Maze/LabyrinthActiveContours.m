%% Generate the labyrinth
% I = maze(5, 5, 'c', 1);   % figure 5-1
I = maze(5, 5, 'c', 3);   % figure 5-7
% I = maze(5, 5, 'c', 5);   % figure 5-8
% I = maze(5, 5, 'c', 4);   % figure 5-12

% I = maze(5, 5, 'c', n);


%% Create the edge map
sigma = 2;
%     smIm = abs(imfilter(Im, fspecial('laplacian'), 'same', 'replicate'));
% f = imfilter(double(I), fspecial('gaussian', ceil(sigma*[3 3]), sigma), 'same', 'replicate');
f = double(I > 0.5);

se = strel('ball',50,50);
f = 50-imdilate(1-f,se);

imsize = size(I);

x1 = 115;
y1 = 65;
x2 = 465;
y2 = 340;

%% Display the figure
hf = figure;
ha(1) = subplot(221);
imshow(I)
title('Original Image')
ha(2) = subplot(222);
hIm = imshow(f,[]);
hold on
title('Energy function with initial spline')    


%% Compute the GVF of the edge map f
tic, [px,py] = GVF(f, 0.2, 80); toc


%% Apply GVF modifications

% Apply a clockwise/counterclockwise rotation around the edges
LeftRight = [zeros(size(f,1),1) diff(f, [], 2)];
UpDown = [zeros(1,size(f,2)); diff(f, [], 1)];
maxEdge = 1;

clockwise = true;
% clockwise = false;
if clockwise 
    py(LeftRight < 0) = -maxEdge/2;    px(LeftRight < 0) = 0;
    py(LeftRight > 0) = maxEdge/2;     px(LeftRight > 0) = 0;
    px(UpDown < 0) = maxEdge/2;
    px(UpDown > 0) = -maxEdge/2;
else
    py(LeftRight < 0) = maxEdge/2;    px(LeftRight < 0) = 0;
    py(LeftRight > 0) = -maxEdge/2;     px(LeftRight > 0) = 0;
    px(UpDown < 0) = -maxEdge/2;
    px(UpDown > 0) = maxEdge/2;
end    

% Apply a slope towards the goal point
towardsGoal = true;
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
newMag = 1 - newMag./max(newMag(:));

ind = I < 0.5;
px(ind) = dx(ind)./newMag(ind);
py(ind) = dy(ind)./newMag(ind);

% Make the magnitude of all vectors equal
magGVF = hypot(px,py) + 1e-10;
px = px./magGVF;
py = py./magGVF;

stepSize = 5;% 1;% 2;% 10;%
[qx,qy] = meshgrid(1:stepSize:imsize(1), 1:stepSize:imsize(2));
ind = sub2ind(imsize, qx, qy);
ha(3) = subplot(223); quiver(qy,qx,px(ind),py(ind)); set(ha(3), 'ydir', 'reverse') %set(gca, 'ydir', 'normal','xdir','reverse')
title(ha(3), 'Gradients')

%% Initialize the snake
t = linspace(0,1, 150)';
x = x1 + t.*(x2-x1);
y = y1 + t.*(y2-y1);

% Draw the initial snake
plot(ha(2), x(:), y(:), 'r', 'linewidth', 3);

% Perform the snake algorithm
ha(4) = subplot(224); cla(ha(4))
imshow(I), hold on, title('Results')

% [x, y] = snakedeform(x,y,0.25,0.5,1,5,px,py,400);
nIter = 10;
corder = jet(nIter );
count = 1;
for n = 1:nIter
%     [x, y] = snakedeform(x,y,0.4,0.1,1,2,px,py,40);
   [x, y] = snakedeform(x,y,1,0.1,1,4,px,py,40);

    plot(ha(4), x(:), y(:), 'Color', corder(count,:), 'linewidth', 1, 'marker', 'none');
    plot([x1 x2], [y1 y2], 'rx', 'MarkerSize', 10);
    count = count + 1;
end

linkaxes(ha, 'xy')
axis(ha(4), 'normal')