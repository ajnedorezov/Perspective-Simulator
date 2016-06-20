%% Read in the image
% I = double(imread('sampleObstacleIPM.png'));
% vpx = 440;
% vpy = 1401;
% cx = 400;
% cy = 1;

I = double(imread('sampleObstacleIPM2.png'));
vpx = 300;
vpy = 1201;
cx = 272;
cy = 1;


%% Convert image to edge map
% f = I;

smArea = 50;
se = strel('ball', smArea, smArea);
GI = imdilate(I, se)-smArea;
f = smArea-GI;
% f(:,[1:4 (end-3:end)]) = 1;
% f([1:4 (end-3:end)], :) = 1;

clf(figure(1)), 
%%
cla(subplot(121)), imagesc(f), set(gca, 'ydir', 'normal','xdir','reverse'), hold on

%% Compute the GVF of the edge map f
tic, [px,py] = GVF(f, 0.2, 40); toc

%%
% p = hypot(px,py);
% maxEdge = max(max(p)); %0.25
% edgeX = maxEdge*px./(p + eps);
% edgeY = maxEdge*py./(p + eps);
% 
% px = px + edgeX/2;
% py = py + edgeY/2;

% Make the magnitude of all vectors equal
magGVF = 2*hypot(px,py) + 1e-10;
px = px./magGVF;
py = py./magGVF;

imsize = size(px);

[cc,rr] = meshgrid(1:imsize(2), 1:imsize(1));
dy = vpy - rr;
dx = vpx - cc;
newMag =  sqrt(dx.*dx + dy.*dy) + eps;

ind = logical(I);
px(ind) = 0.25*dx(ind)./newMag(ind);
py(ind) = 0.25*dy(ind)./newMag(ind);

[qx,qy] = meshgrid(1:10:imsize(1), 1:10:imsize(2));
ind = sub2ind(imsize, qx,qy);
subplot(122), quiver(qy,qx,px(ind),py(ind)); set(gca, 'ydir', 'normal','xdir','reverse')

%% Initialize the snake
t = linspace(0,1, 100)';
% t = [t; flipud(t)];
x = cx + t.*(vpx-cx); % vpx.*t + (1-t).*cx;
y = cy + t.*(vpy-cy);

cla(subplot(121)), imagesc(f), set(gca, 'ydir', 'normal','xdir','reverse'), hold on

subplot(121), hold on,
% snakedisp(x,y,'r');
% plot(x(:), y(:), 'r', 'linewidth', 3);


% [x,y] = snakedeform(x,y,0.32,0.25,0.0375,25,px,py,5*5);   % sample image 2
[x,y] = snakedeform(x,y,0.32,0.25,0.375,25,px,py,5*20);   % sample image 2

% [x,y] = snakedeform(x,y,1,0.75,0.5,25,px,py,5*5); % sample image 1

% [x,y] = snakedeform(x,y,0.5,0.5,0.5,50,px,py,5*5);

% [x,y] = snakedeform(x,y,0.05,10,1,60,px,py,5*5);
% [x,y] = snakedeform(x,y,0.075,43,1,90,px,py,5*50);
% snakedisp(x,y,'m');
plot(x(:), y(:), 'Color', [1 0 0], 'linewidth', 4);
plot([cx vpx], [cy vpy], 'rx', 'MarkerSize', 10);


