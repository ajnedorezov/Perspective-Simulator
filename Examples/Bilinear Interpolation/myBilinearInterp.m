% %%
% ox = 1:100;
% oy = 1:100;
% [x,y] = meshgrid(ox, oy);
% 
% im = hypot(x,y);
% im(x<=50 & y <=50) = 0;
% im(x>50 | y > 50) = 100;
% 
% clf(figure(1)), imagesc(im)
% 
% % [nx,ny] = meshgrid(1:0.5:100, 1:0.5:100);
% % [nx,ny] = meshgrid(1:2:100, 1:2:100);
% [nx,ny] = meshgrid([1:25 26:0.5:50 55:5:100], [1:100]);

%%
im = double(imread('freeway.jpg'));
origIm = im;
[n,m,~] = size(im);

ox = 1:m;
oy = 1:n;

[x,y] = meshgrid(1:m, 1:n);
% [nx,ny] = meshgrid(1:2:m, 1:2:n);
[nx,ny] = meshgrid(1:0.25:m, 1:0.25:n);

%% Find the for neighboring points
% b = bsxfun(@minus, ox, nx(:));
% b1 = b;
% b2 = b;
% b1(b>0) = nan;
% b2(b<=0) = nan;
% 
% [~,x1] = min(abs(b1), [], 2);
% [~,x2] = min(abs(b2), [], 2);
% 
% c = bsxfun(@minus, oy, ny(:));
% c1 = c;
% c2 = c;
% c1(c>0) = nan;
% c2(c<=0) = nan;
% 
% [~,y1] = min(abs(c1), [], 2);
% [~,y2] = min(abs(c2), [], 2);
% 
% x1 = reshape(x1, size(nx));
% x2 = reshape(x2, size(nx));
% y1 = reshape(y1, size(ny));
% y2 = reshape(y2, size(ny));

x1 = ones(size(nx));
x2 = ones(size(nx));
y1 = ones(size(ny));
y2 = ones(size(ny));
for r = 1:size(nx,1)
    for c = 1:size(nx,2)
        b = ox - nx(r,c);
        b1 = b;
        b2 = b;
        b1(b>0) = nan;
        b2(b<=0) = nan;

        [~,x1(r,c)] = min(abs(b1), [], 2);
        [~,x2(r,c)] = min(abs(b2), [], 2);

        b = oy - ny(r,c);
        b1 = b;
        b2 = b;
        b1(b>0) = nan;
        b2(b<=0) = nan;

        [~,y1(r,c)] = min(abs(b1), [], 2);
        [~,y2(r,c)] = min(abs(b2), [], 2);
    end
end

% getX = @(a,b) [x1(a,b) y1(a,b) x2(a,b) y2(a,b)];
% 
% d = getX(20,20);

%% We have the indicies, now let's find the weights

ind00 = sub2ind(size(x), y1, x1);
ind01 = sub2ind(size(x), y1, x2);
ind10 = sub2ind(size(y), y2, x1);
ind11 = sub2ind(size(y), y2, x2);

% xx = 1-(x2-nx)./(x2-x1);
% yy = 1-(y2-ny)./(y2-y1);

denom = (x2-x1).*(y2-y1);
b11 = (x2-nx).*(y2-ny)./denom;
b12 = (x2-nx).*(ny-y1)./denom;
b21 = (nx-x1).*(y2-ny)./denom;
b22 = (nx-x1).*(ny-y1)./denom;

%%
% newIm = im(ind00).*(1-xx).*(1-yy) ...
%             + im(ind01).*(1-xx).*yy ...
%             + im(ind10).*xx.*(1-yy) ...
%             + im(ind11).*xx.*yy;

newIm = zeros([size(nx),3]);
for n = 1:3
    im = origIm(:,:,n);
    newIm(:,:,n) = im(ind00).*b11 ...
            + im(ind01).*b21 ...
            + im(ind10).*b12 ...
            + im(ind11).*b22;
end
    
clf(figure(2)), imshow(uint8(newIm))
