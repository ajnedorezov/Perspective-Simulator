%% Load in the image
% im = imread('BaseIm.png');
% im = imresize(im, 0.5);
% im = im(240:end, :, :);
vid = VideoReader('Downsampled To Work Video.avi');
Ic = readFrame(vid);
Ic = rgb2gray(Ic);
im = double(Ic)/255;
imsize = size(im);

% Display the image
clf(figure(1))
imshow(im)

% Get the 4 corners of a lane marker (starting with bottom left corner & clockwise)
% axis([300 340 380 465])
% pts = round(ginput(4));
% pts = [306 462;
%        312 389;
%        330 389;
%        335 462];
% pts = [64 230;
%        130 155;
%        191 155;
%        257 230];
pts = [0, imsize(1);
       imsize(2)/2-90 330;
       imsize(2)/2+90 330;
       imsize(2) imsize(1)];
% pts(:,2) = pts(:,2) - 240;

% pts = bsxfun(@rdivide, pts, imsize([2 1]));
   
newPts = [pts(1,:);
          pts(1,1) pts(2,2);
          pts(4,1) pts(3,2);
          pts(4,:)];
% newPts = [pts(1,:);
%           pts(1,1) pts(1,2)-50;
%           pts(1,1)+50 pts(1,2)-50;
%           pts(1,1)+50 pts(1,2)];
      
hold on
plot(pts(:,1), pts(:,2),'o-', newPts(:,1), newPts(:,2), 'x-')
% plot(pts(:,1)*imsize(2), pts(:,2)*imsize(1),'o-', newPts(:,1)*imsize(2), newPts(:,2)*imsize(1), 'x-')
hold off

% keyboard

%%   
% % Solve the A transformation matrix
% % A = [x y 1 0 0 0 -x*x' -y*y' -x';
% %      0 0 0 x y 1 -x*y' -y*y' -y'];
% 
% A = [pts ones(4,1) zeros(4,3) -pts(:,1).*newPts(:,1) -pts(:,2).*newPts(:,1) -newPts(:,1);
%      zeros(4,3) pts ones(4,1) -pts(:,1).*newPts(:,2) -pts(:,2).*newPts(:,2) -newPts(:,2)];

% x = newPts(:, 1)'; y = newPts(:,2)'; X = pts(:,1)'; Y = pts(:,2)';
% n = size(pts,1);
% rows0 = zeros(3, n);
% rowsXY = -[X; Y; ones(1,n)];
% hx = [rowsXY; rows0; x.*X; x.*Y; x];
% hy = [rows0; rowsXY; y.*X; y.*Y; y];
% h = [hx hy];
% [U,~,~] = svd(h);
% 
% H = reshape(U(:,9), 3, 3);

%  
% x = linsolve(A(:,1:8), -A(:,9));
% tform = reshape([x;1], 3,3)';

% tform = maketform('projective', pts, newPts);
% tform = maketform('projective', newPts, pts);

% tform = fitgeotrans(newPts,pts,'projective');
% tform = fitgeotrans(pts,newPts,'projective');

%%
theta = -80;
h = 1.5;
ku = imsize(2)/imsize(1);
kv = 1;
u = imsize(2);
v = imsize(1);
s = 0;

R = [1 0 0 0;
     0 cosd(theta) -sind(theta) 0;
     0 sind(theta) cosd(theta) 0;
     0 0 0 1];
T = eye(4);
T(3,4) = -h/sind(theta);
K = [ku s 0 0;
     0 kv 0 0;
     0 0 1 0];
 
H = K*T*R;

% [uu,vv] = meshgrid(linspace(-1,1), linspace(-1,1));
ds = 50;
[uu,vv] = meshgrid(-round(imsize(2)/2):ds:round(imsize(2)/2), (round(imsize(1)/2)+1):ds:imsize(1));

tempU = uu(:)';
tempV = vv(:)';
% tempIm = H*[tempU;tempV;ones(size(tempU))];
tempIm = H*[tempU;tempV;zeros(size(tempU));ones(size(tempU))];
p = tempIm(3,:);
tempIm = bsxfun(@rdivide, tempIm, p);
nnx = reshape(tempIm(1,:), size(uu));
nny = reshape(tempIm(2,:), size(vv));
clf(figure(2)), plot(uu,vv,'o'), axis ij, axis([-480 480 0 540])
clf(figure(3)), plot(nnx,nny,'x')

%%
[xx,yy] = meshgrid(-round(imsize(2)/2):ds:round(imsize(2)/2), (round(imsize(1)/2)+1):ds:imsize(1));
b = interp2(xx, yy, double(Ic), nnx,nny);

% %%
% [newX,newY] = meshgrid(-40:5:40,0:10:300);
% % newX = newX/40;
% % newY = newY/300;
% 
% tempY = newY(:)';
% tempX = newX(:)';
% tempIm = tform.T*[tempX;tempY;ones(size(tempX))];
% nnx = reshape(tempIm(1,:), 31, 17);
% nny = reshape(tempIm(2,:), 31, 17);
% figure, plot(newX,newY,'o')
% figure, plot(nnx,nny,'x')

%% Apply the transformation to the image
% b = imtransform(im, tform);
% b = imwarp(im, tform);

%% display the new image
clf(figure(2))
% imshowpair(im, b, 'montage')
% image(b(450:end,15000:22000,:))
imshow(b)
