%% Load in the image
im = imread('BaseIm.png');
% im = imresize(im, 0.5);
im = im(240:end, :, :);
% Display the image
clf(figure(1))
image(im)

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
pts = [128 460;
       309 250;
       332 250;
       513 460];
pts(:,2) = pts(:,2) - 240;
   
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
hold off
   
% % Solve the A transformation matrix
% % A = [x y 1 0 0 0 -x*x' -y*y' -x';
% %      0 0 0 x y 1 -x*y' -y*y' -y'];
% 
% A = [pts ones(4,1) zeros(4,3) -pts(:,1).*newPts(:,1) -pts(:,2).*newPts(:,1) -newPts(:,1);
%      zeros(4,3) pts ones(4,1) -pts(:,1).*newPts(:,2) -pts(:,2).*newPts(:,2) -newPts(:,2)];
%  
% x = linsolve(A(:,1:8), -A(:,9));
% tform = reshape([x;1], 3,3)';

% tform = maketform('projective', pts, newPts);
% tform = maketform('projective', newPts, pts);

% tform = fitgeotrans(newPts,pts,'projective');
tform = fitgeotrans(pts,newPts,'projective');

%% Apply the transformation to the image
% b = imtransform(im, tform);
b = imwarp(im, tform);

%% display the new image
clf(figure(2))
% imshowpair(im, b, 'montage')
image(b(450:end,15000:22000,:))
