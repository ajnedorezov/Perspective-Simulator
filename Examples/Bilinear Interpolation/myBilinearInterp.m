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
% [x,y] = meshgrid(fliplr(m./(1:m)), fliplr(n./(1:n)));
% [nx,ny] = meshgrid(1:2:m, 1:2:n);
% [nx,ny] = meshgrid(1:0.25:m, 1:0.25:n);
% nx = 1:0.25:m;
% ny = 1:0.25:n;
nx = 1:0.5:m;
ny = 1:0.5:n;
% nx = 1:2.5:m;
% ny = 1:2.5:n;
% nx = 1:2:m;
% ny = 1:2:n;
% 
% I = im(:,:,2);
% F = scatteredInterpolant(x(:),y(:),I(:), ...
%         'linear', 'none');
% tic, newI = F(nx,ny); toc
    
% tI = im(:,:,1);
% for r = 1:length(ny)
%     % http://www.mathworks.com/matlabcentral/fileexchange/25443-interpolation-for-missing-data/content/interpolation/lininter.m
%     for c = 1:length(nx)
%         I(r,c) = lininter(x(:), y(:), tI(:), nx(c), ny(r));
%     end
%     disp(['Row: ' num2str(r)])
% end


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
for r = 1:length(ny)
    for c = 1:length(nx)
        % Find the 4 closest point
        d = (x(:) - nx(c)).^2 + (y(:) - ny(r)).^2;
%         d = (nx(c) - x).^2 + (ny(r) - y).^2;
%         dx = x - nx(c);
%         dy = y - ny(r);
%         d = (dx).^2 + (dy).^2;
        
%         tInd(1) = find(dx <= 0 & dy <= 0, 1, 'last');
%         tInd(2) = tInd(1)+1;%find(dx <= 0 & dy > 0, 1, 'first');
% %         tInd(3) = find(dx > 0 & dy <= 0, 1, 'first');
%         tInd(4) = find(dx > 0 & dy > 0, 1, 'first');
%         tInd(3) = tInd(4)-1;
        tInd = [];
%         for n = 1:4
%             [~, ind] = min (d);
%             
%             tInd(n) = ind;
%             d(ind) = inf;
%         end
%         [tx,ty] = ind2sub(size(x),tInd);

        [~, ind] = min (d);

        tInd(1) = ind;
        % Get the adjacent pixels
        isTop = mod(tInd(1), n) == 0;
        isBottom = mod(tInd(1), n) == 1;
        isLeft = tInd(1) <= n;
        isRight = tInd(1) > n*(m-1);
        
        inds = nan(1,9);
        if ~isTop
            % Not at the top edge
            if ~isLeft, inds(1) = tInd(1)+1-n; end
            inds(2) = tInd(1)+1;
            if ~isRight, inds(3) = tInd(1)+1+n; end
        end
        if ~isLeft, inds(4) = tInd(1)-n; end
        if ~isRight, inds(6) = tInd(1)+n; end
        if ~isBottom
            % Not at the top edge
            if ~isLeft, inds(7) = tInd(1)-1-n; end
            inds(8) = tInd(1)-1;
            if ~isRight, inds(9) = tInd(1)-1+n; end
        end
        minds = inds(~isnan(inds));
        dist = d(minds);
        for q = 2:3;
            [~,i] = min(dist);
            tInd(q) = minds(i);
            dist(i) = inf;
        end
        
        tx = x(tInd)';
        ty = y(tInd)';
%         F = [ones(4,1) tx ty tx.*ty ];
%         if det(F) < eps
%             A = [1;0;0;0];
%         else
%             A = F\[1 nx(c) ny(r) nx(c)*ny(r)]';
%         end
        
%         q11 = find(tx <= nx(c) & ty <= ny(r), 1, 'first');
%         q12 = find(tx <= nx(c) & ty > ny(r), 1, 'first');
%         q21 = find(tx > nx(c) & ty <= ny(r), 1, 'first');
%         q22 = find(tx > nx(c) & ty > ny(r), 1, 'first');
% 
%         tInd = tInd([q11 q12 q21 q22]);
%         tx = x(tInd)';
%         ty = y(tInd)';
        
        ind00(r,c) = tInd(1);
        ind01(r,c) = tInd(2);
        ind10(r,c) = tInd(3);
        ind11(r,c) = tInd(4);
        
        
      

        ddx = (max(tx)-min(tx));
        ddy = (max(ty)-min(ty));
        denom = ddx*ddy;
        dx = ddx-abs((tx-nx(c)));
        dy = ddy-abs((ty-ny(r)));
        tf = dx<eps&dy<eps;
%         if sum(tf) == 1
%             A = double(tf);
%         else
            A = dx.*dy./denom;
%         end
        A = A./norm(A);
        b11(r,c) = A(1);
        b12(r,c) = A(2);
        b21(r,c) = A(3);
        b22(r,c) = A(4);

%         x1(r,c) = min(tx);
%         x2(r,c) = max(tx);
%         y1(r,c) = min(ty);
%         y2(r,c) = max(ty);
    end
    
%     for c = 1:length(nx)
%         dx = x(:)-nx(c);
%         dy = y(:)-ny(r);
%         d = dx.^2 + dy.^2;
%         [~,ind] = min(d(:));
%         [tr,tc] = ind2sub(size(x), ind);
%         
%         if nx(c)-x(ind) >= 0
%             x1(r,c) = tc;
%             x2(r,c) = tc + 1;
%         else
%             x1(r,c) = tc - 1;
%             x2(r,c) = tc;
%         end
%         if ny(r)-y(ind) >= 0
%             y1(r,c) = tr;
%             y2(r,c) = tr + 1;
%         else
%             y1(r,c) = tr - 1;
%             y2(r,c) = tr;
%         end
%     end
%     disp(['Row: ' num2str(r)])
%     toc

%         dx = bsxfun(@minus, x(:), nx);
%         dy = y(:)-ny(r);
%         d = bsxfun(@plus, dx.^2,  dy.^2);
%         [~,ind] = min(d,[],1);
%         [tr,tc] = ind2sub(size(x), ind);
%         
%         tfAbove = nx-x(ind) >= 0;
%             if any(tfAbove)
%                 x1(r,tfAbove) = tc(tfAbove);
%                 x2(r,tfAbove) = tc(tfAbove) + 1;
%             end
%             if any(~tfAbove)
%                 x1(r,~tfAbove) = tc(~tfAbove) - 1;
%                 x2(r,~tfAbove) = tc(~tfAbove);
%             end
%             
%         tfAbove = ny(r)-y(ind) >= 0;
%             if any(tfAbove)
%                 y1(r,tfAbove) = tr(tfAbove);
%                 y2(r,tfAbove) = tr(tfAbove) + 1;
%             end
%             if any(~tfAbove)
%                 y1(r,~tfAbove) = tr(~tfAbove) - 1;
%                 y2(r,~tfAbove) = tr(~tfAbove);
%             end
        
%         if nx(c)-x(ind) >= 0
%             x1(r,:) = tc;
%             x2(r,:) = tc + 1;
%         else
%             x1(r,:) = tc - 1;
%             x2(r,:) = tc;
%         end
%         if ny(r)-y(ind) >= 0
%             y1(r,c) = tr;
%             y2(r,c) = tr + 1;
%         else
%             y1(r,c) = tr - 1;
%             y2(r,c) = tr;
%         end
    disp(['Row: ' num2str(r)])

end
toc
% x1(x1<1) = 1;
% x1(x1>size(x,2)) = size(x,2);
% x2(x2<1) = 1;
% x2(x2>size(x,2)) = size(x,2);
% y1(y1<1) = 1;
% y1(y1>size(x,1)) = size(x,1);
% y2(y2<1) = 1;
% y2(y2>size(x,1)) = size(x,1);

% keyboard
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

% x1 = ones(size(nx));
% x2 = ones(size(nx));
% y1 = ones(size(ny));
% y2 = ones(size(ny));
% for r = 1:size(nx,1)
%     for c = 1:size(nx,2)
%         b = ox - nx(r,c);
%         b1 = b;
%         b2 = b;
%         b1(b>0) = nan;
%         b2(b<=0) = nan;
% 
%         [~,x1(r,c)] = min(abs(b1), [], 2);
%         [~,x2(r,c)] = min(abs(b2), [], 2);
% 
%         b = oy - ny(r,c);
%         b1 = b;
%         b2 = b;
%         b1(b>0) = nan;
%         b2(b<=0) = nan;
% 
%         [~,y1(r,c)] = min(abs(b1), [], 2);
%         [~,y2(r,c)] = min(abs(b2), [], 2);
%     end
% end

% getX = @(a,b) [x1(a,b) y1(a,b) x2(a,b) y2(a,b)];
% 
% d = getX(20,20);

% %% We have the indicies, now let's find the weights
% 
% % ind00 = sub2ind(size(x), y1, x1);
% % ind01 = sub2ind(size(x), y1, x2);
% % ind10 = sub2ind(size(y), y2, x1);
% % ind11 = sub2ind(size(y), y2, x2);
% % ind00 = sub2ind(size(x), x1, y1);
% % ind01 = sub2ind(size(x), x1, y2);
% % ind10 = sub2ind(size(y), x2, y1);
% % ind11 = sub2ind(size(y), x2, y2);
% 
% % xx = 1-(x2-nx)./(x2-x1);
% % yy = 1-(y2-ny)./(y2-y1);
% 
% denom = (x2-x1).*(y2-y1);
% % b11 = (x2-nx).*(y2-ny)./denom;
% % b12 = (x2-nx).*(ny-y1)./denom;
% % b21 = (nx-x1).*(y2-ny)./denom;
% % b22 = (nx-x1).*(ny-y1)./denom;
% b11 = bsxfun(@minus, x2, nx).* bsxfun(@minus, y2, ny')./denom;
% b12 = bsxfun(@minus, x2, nx).* bsxfun(@minus, ny', y1)./denom;
% b21 = bsxfun(@minus, nx, x1).* bsxfun(@minus, y2, ny')./denom;
% b22 = bsxfun(@minus, nx, x1).* bsxfun(@minus, ny', y1)./denom;
%%
% newIm = im(ind00).*(1-xx).*(1-yy) ...
%             + im(ind01).*(1-xx).*yy ...
%             + im(ind10).*xx.*(1-yy) ...
%             + im(ind11).*xx.*yy;

newIm = zeros([length(ny) length(nx),3]);
for n = 1:3
    im = origIm(:,:,n);
    newIm(:,:,n) = im(ind00).*b11 ...
            + im(ind01).*b21 ...
            + im(ind10).*b12 ...
            + im(ind11).*b22;
end
    
clf(figure(2)), imshow(uint8(newIm))
