%%
im = double(imread('freeway.jpg'));
origIm = im;
[n,m,~] = size(im);

ox = 1:m;
oy = 1:n;

[x,y] = meshgrid(1:m, 1:n);
% [x,y] = meshgrid(fliplr(m./(1:m)), fliplr(n./(1:n)));
% [nx,ny] = meshgrid(1:2:m, 1:2:n);
% [nx,ny] = meshgrid(1:0.25:m, 1:0.25:n);
nx = 1:0.25:m;
ny = 1:0.25:n;
% nx = 1:2.5:m;
% ny = 1:2.5:n;

%% Find the 3 neighboring points
ind00 = ones(length(ny), length(nx));
ind01 = ind00;
ind10 = ind00;
b11 = ones(length(ny), length(nx));
b12 = b11;
b21 = b11;
tic
for r = 1:length(ny)
    for c = 1:length(nx)
        % Find the 3 closest point
        d = (x(:) - nx(c)).^2 + (y(:) - ny(r)).^2;

        tInd = [];
        tDist = [];
        for n = 1:3
            [tDist(n), tInd(n)] = min (d);
            
            d(tInd(n)) = inf;
        end
        
        tx = x(tInd)';
        ty = y(tInd)';
        denom = max(max(hypot(bsxfun(@minus, tx, tx'), bsxfun(@minus, ty, ty'))));
        
        ind00(r,c) = tInd(1);
        ind01(r,c) = tInd(2);
        ind10(r,c) = tInd(3);
        
        F = [tx ty ones(3,1)];
        A = F\[nx(c) ny(r) 1]';
        b11(r,c) = A(1);
        b12(r,c) = A(2);
        b21(r,c) = A(3);
    end
    disp(['Row: ' num2str(r)])

end
toc

%%
newIm = zeros([length(ny) length(nx),3]);
for n = 1:3
    im = origIm(:,:,n);
    newIm(:,:,n) = im(ind00).*b11 ...
            + im(ind01).*b21 ...
            + im(ind10).*b12;
end
    
clf(figure(2)), imshow(uint8(newIm))
