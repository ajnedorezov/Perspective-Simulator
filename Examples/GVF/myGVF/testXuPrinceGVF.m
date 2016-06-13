%% Read in the image
I = double(imread('sampleObstacleIPM.png'));
vpx = 430;
vpy = 1401;
cx = 400;
cy = 1;

%% Convert image to edge map
% f = I;

smArea = 50;
se = strel('ball', smArea, smArea);
GI = imdilate(I, se);
f = 1-I;

clf(figure(1)), 
%%
cla(subplot(121)), imagesc(GI), set(gca, 'ydir', 'normal','xdir','reverse'), hold on

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
% 
magGVF = hypot(px,py) + 1e-10;
px = px./magGVF;
py = py./magGVF;

imsize = size(px);
[xx,yy] = meshgrid(1:10:imsize(1), 1:10:imsize(2));
ind = sub2ind(imsize, xx,yy);
subplot(122), quiver(yy,xx,px(ind),py(ind)); set(gca, 'ydir', 'normal','xdir','reverse')

%% Initialize the snake
t = linspace(0,1, 50)';
t = [t; flipud(t)];
x = cx + t.*(vpx-cx); % vpx.*t + (1-t).*cx;
y = cy + t.*(vpy-cy);

cla(subplot(121)), imagesc(GI), set(gca, 'ydir', 'normal','xdir','reverse'), hold on

subplot(121), hold on,
snakedisp(x,y,'r');

[x,y] = snakedeform(x,y,0.05,5,1,50,px,py,5*5);
% [x,y] = snakedeform(x,y,0.075,43,1,90,px,py,5*50);
snakedisp(x,y,'m');


