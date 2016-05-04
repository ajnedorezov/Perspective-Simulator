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

% origIm = double(imread('BaseIm.png'));
% imsize = size(origIm);


myIPM = IPM_vert(imsize(1:2),...
        'cameraZ', 7,...
        'theta', eps,...0.003125*pi/180,...atan(1/(2*1609.34)),...
        'stepSize', [.5 1],...[.25 .5],...[.125 .25],...
        'xRange', [-50 50],...
        'yRange', [0 350]);

x = myIPM.XWorld;
y = myIPM.YWorld;
nx = myIPM.xSteps;
ny = myIPM.ySteps;

[n,m] = size(x);

% nx = 1:0.5:m;
% ny = 1:0.5:n;
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
% 

%% Find the 3 neighboring points
% x1 = ones(length(ny), length(nx));
% x2 = ones(length(ny), length(nx));
% y1 = ones(length(ny), length(nx));
% y2 = ones(length(ny), length(nx));
ind00 = ones(length(ny), length(nx));
ind01 = ind00;
ind10 = ind00;
ind11 = ind00;
b11 = zeros(length(ny), length(nx));
b12 = b11;
b21 = b11;
b22 = b11;
tic

TRI = delaunay(x,y);
for r = 1:length(ny)
    for c = 1:length(nx)
        % Find the triangles touching the closest point
        d = (x(:) - nx(c)).^2 + (y(:) - ny(r)).^2;
        [~, ind] = min(d);
        
        triInd = find(any(ismember(TRI, ind), 2));
        
        for t = triInd'
            % Loop over the different triangles and find the one that
            % contains this point
            
            tx = x(TRI(t,:));
            ty = y(TRI(t,:));

            A = [ones(1,3);
                 tx
                 ty];
            
            Ai = A; Ai(2:3,1) = [nx(c);ny(r)];
            Aj = A; Aj(2:3,2) = [nx(c);ny(r)];
            Ak = A; Ak(2:3,3) = [nx(c);ny(r)];
            detA = det(A);
            w1 = det(Ai)/detA;
            w2 = det(Aj)/detA;
            w3 = det(Ak)/detA;
            
            if all(sign([w1 w2 w3]) >= 0) || all(sign([w1 w2 w3]) <= 0)
                ind00(r,c) = TRI(t,1);
                ind01(r,c) = TRI(t,1);
                ind10(r,c) = TRI(t,1);
                b11(r,c) = w1;
                b12(r,c) = w2;
                b21(r,c) = w3;
                break
            end
        end
        
        
%         % Find the 4 closest point
%         d = (x(:) - nx(c)).^2 + (y(:) - ny(r)).^2;
% %         d = (nx(c) - x).^2 + (ny(r) - y).^2;
% %         dx = x - nx(c);
% %         dy = y - ny(r);
% %         d = (dx).^2 + (dy).^2;
%         
% %         tInd = [];
% %         count = 1;
% %         iter = 0;
% %         while count <= 3
% %             iter = iter + 1;
% %             if iter >= 100;
% %                 break;
% %             end
% %             
% %             [~, ind] = min(d);
% %             
% %             if count <= 2
% %                 tInd(count) = ind;
% %                 d(ind) = inf;
% %                 count = count + 1;
% %             else
% %                 % Need to look for a 3rd point that will make a triangle
% %                 tInd(count) = ind;
% %                 d(ind) = inf;
% %                 
% % %                 if any(tInd - ind > 2*max(imsize))
% % %                     % Nothing near the surrounding area, time to move on
% % %                     break
% % %                 end
% %                 
% %                 tx = x(tInd);
% %                 ty = y(tInd);
% % 
% %                 A = [ones(1,3);
% %                      tx
% %                      ty];
% %                 if det(A) > 1e-3
% %                     % The 3 points are not colinear
% %                     count = count + 1;                    
% %                 end
% %             end
% %         end
% %         if count < 4
% %             % Was not able to find a triangle
% %             continue
% %         end
% %         
% % %         [tx,ty] = ind2sub(size(x),tInd);
% %         
% %         ind00(r,c) = tInd(1);
% %         ind01(r,c) = tInd(2);
% %         ind10(r,c) = tInd(3);
% % %         ind11(r,c) = tInd(4);
% % 
% %         tx = x(tInd);
% %         ty = y(tInd);
% %         
% %         A = [ones(1,3);
% %              tx
% %              ty];
% %         if det(A) < 1e-3
% %             % The 3 points are colinear so weight this point as zeros
% %             continue
% %         end
% %         
% %         Ai = A; Ai(2:3,1) = [nx(c);ny(r)];
% %         Aj = A; Aj(2:3,2) = [nx(c);ny(r)];
% %         Ak = A; Ak(2:3,3) = [nx(c);ny(r)];
% %         detA = det(A);
% %         w1 = det(Ai)/detA;
% %         w2 = det(Aj)/detA;
% %         w3 = det(Ak)/detA;
% %         
% %         if all(sign([w1 w2 w3]) > 0) || all(sign([w1 w2 w3]) < 0) %sum(abs([w1 w2 w3])) <= 1.1
% %             % Point exists inside triangle soooo save the weights
% %             b11(r,c) = w1;
% %             b12(r,c) = w2;
% %             b21(r,c) = w3;
% %         else
% %             1;
% %         end
%         
% % %         detIJK = (tx(2)-tx(1))*(ty(3)-ty(1)) - (tx(3)-tx(1))*(ty(2)-ty(1));
% % %         
% % %         b11(r,c) = ((tx(2)-nx(c))*(ty(3)-ny(r)) - (tx(3)-nx(c))*(ty(2)-ny(r)))/detIJK;
% % %         b12(r,c) = ((tx(3)-nx(c))*(ty(1)-ny(r)) - (tx(1)-nx(c))*(ty(3)-ny(r)))/detIJK;
% % %         b21(r,c) = ((tx(1)-nx(c))*(ty(2)-ny(r)) - (tx(2)-nx(c))*(ty(1)-ny(r)))/detIJK;
%         
    end
    
    disp(['Row: ' num2str(r)])

end
toc

%% Interpolate the new image

newIm = zeros([length(ny) length(nx),3]);
for n = 1:3
    im = origIm(myIPM.rHorizon:end,:,n);
    newIm(:,:,n) = im(ind00).*b11 ...
            + im(ind01).*b12 ...
            + im(ind10).*b21;
end
    
clf(figure(2)), imshow(fliplr(flipud(uint8(newIm))))
