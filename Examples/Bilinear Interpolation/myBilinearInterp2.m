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
% im = double(imread('freeway.jpg'));
% origIm = im;

% 
% ox = 1:m;
% oy = 1:n;

% [x,y] = meshgrid(1:m, 1:n);
% [x,y] = meshgrid(fliplr(m./(1:m)), fliplr(n./(1:n)));
% [nx,ny] = meshgrid(1:2:m, 1:2:n);
% [nx,ny] = meshgrid(1:0.25:m, 1:0.25:n);
% nx = 1:0.25:m;
% ny = 1:0.25:n;

vid = VideoReader('Downsampled To Work Video.avi');
origIm = double(readFrame(vid));
imsize = size(origIm);


myIPM = IPM_vert(imsize(1:2),...
        'cameraZ', 7,...
        'theta', eps,...0.003125*pi/180,...atan(1/(2*1609.34)),...
        'stepSize', [1 1],...[.25 .5],...[.125 .25],...
        'xRange', [-50 50],...
        'yRange', [0 350]);

x = myIPM.XWorld;
y = myIPM.YWorld;
nx = myIPM.xSteps;
ny = myIPM.ySteps;

[n,m] = size(x);

%% Find the four neighboring points
% x1 = ones(length(ny), length(nx));
% x2 = ones(length(ny), length(nx));
% y1 = ones(length(ny), length(nx));
% y2 = ones(length(ny), length(nx));
ind00 = ones(length(ny), length(nx));
ind01 = ind00;
ind10 = ind00;
ind11 = ind00;
b11 = ones(length(ny), length(nx));
b12 = b11;
b21 = b11;
b22 = b11;
tic

TRI = delaunay(x,y);
for r = 1:length(ny)
    for c = 1:length(nx)
        % Find the closest point
        d = (x(:) - nx(c)).^2 + (y(:) - ny(r)).^2;
        [~, ind] = min(d);

        tInd(1) = ind;
        % Get the adjacent pixels
        isTop = mod(tInd(1), n) == 0;
        isBottom = mod(tInd(1), n) == 1;
        isLeft = tInd(1) <= n;
        isRight = tInd(1) > n*(m-1);
        
        if isTop
            tInd(1) = tInd(1)-1;
        end
        if isRight
            tInd(1) = tInd(1)-n;
        end
        if isLeft
            tInd(1) = tInd(1)+n;
        end
        
        [pr,pc] = ind2sub(size(x),tInd);
        dx = nx(c)-pc;
        dy = ny(r)-pr;
        
        if dx >= 0 
            if  dy >=0
                % Lower left corner
                tInd(2) = tInd(1)+1;
                tInd(3) = tInd(1)+n;
                tInd(4) = tInd(1)+n+1;
            else
                % Upper left corner
                tInd(2) = tInd(1)-1;
                tInd(3) = tInd(1)+n;
                tInd(4) = tInd(1)+n-1;
            end
        else
            if  dy >=0
                % Lower right corner
                tInd(2) = tInd(1)+1;
                tInd(3) = tInd(1)-n;
                tInd(4) = tInd(1)-n+1;
            else
                % Upper right corner
                tInd(2) = tInd(1)-1;
                tInd(3) = tInd(1)-n;
                tInd(4) = tInd(1)-n-1;
            end
        end
            
        tx = x(tInd)';
        ty = y(tInd)';
        
        ind00(r,c) = tInd(1);
        ind01(r,c) = tInd(2);
        ind10(r,c) = tInd(3);
        ind11(r,c) = tInd(4);
        
        ddx = (max(tx)-min(tx));
        ddy = (max(ty)-min(ty));
        denom = ddx*ddy;
        dx = ddx-abs((tx-nx(c)));
        dy = ddy-abs((ty-ny(r)));
%         if all(sign(dx) > 0) || all(sign(dx) < 0) || all(sign(dy) > 0) || all(sign(dy) < 0)
%             A = zeros(1,4);
%         else
            A = dx.*dy./denom;
%         end
        A = A./norm(A);
        b11(r,c) = A(1);
        b12(r,c) = A(2);
        b21(r,c) = A(3);
        b22(r,c) = A(4);
    end
    
    disp(['Row: ' num2str(r)])

end
toc

%%
newIm = zeros([length(ny) length(nx),3]);
for n = 1:3
    im = origIm(myIPM.rHorizon:end,:,n);
    newIm(:,:,n) = im(ind00).*b11 ...
            + im(ind01).*b12 ...
            + im(ind10).*b21 ...
            + im(ind11).*b22;
end
    
clf(figure(2)), imshow(rot90(uint8(newIm),2))


newIm = zeros([length(ny) length(nx),3]);
for n = 1:3
    im = origIm(myIPM.rHorizon:end,:,n);
    newIm(:,:,n) = im(ind00).*b11 ...
            + im(ind01).*b12 ...
            + im(ind10).*b21 ...
            + im(ind11).*b22;
end
    
clf(figure(3)), imshow(rot90(uint8(newIm),2))
