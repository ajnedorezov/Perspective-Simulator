% Script which maps the MATLAB test image in camera pixel coordinates (Ic)
% to a bird's eye view image in world coordinates (Iw) using the inverse
% perspective mapping functions published by Bertozzi and Broggi.
%
% Authors:
%---------
% Eric Johnson and Randy Hamburger
% University of Utah
% CS 5320/6320 - Computer Vision 
% March 8, 2007
%
% Results: The equations work well enough to give a pretty good output
% image, but as found in the Mathcad experimentations, horizontal lines
% come out curved, indicating that there is some kind of inaccuracy/error
% in these equations.  This is worrisome, because we don't know how much
% error in the lane detection this problem may introduce.
% 
% Furthermore, the curvature introduced by these equations makes it so that
% the interpolation in the image->world mapped pixels has to work on a
% totally non-uniformly spaced grid.  This is much more computationally
% intensive than what would be required by a correct mapping where at least
% rows of pixels would remain straight when mapped to world coordinates.

clear; clc;

m = 240;
n = 320;

% Ic = imread('grayRoadScene.tif');
% Ic = imread('roadPicBig.tif');
Ic = imread('BaseIm.png');
% Ic = imread('freeway.jpg');
% Ic = rgb2gray(Ic);
% [m,n] = size(Ic);
% Ic = imresize(Ic, [m, n]);

[m,n,c] = size(Ic);
Ic = double(Ic)/255;

% Obtain alpha from the overall viewing angle.
alpha_tot = 20*pi/180;
den = sqrt((m-1)^2+(n-1)^2);
alpha = atan( (m-1)/den * tan(alpha_tot) );

% cx = 4;
% cy = -10;
% cz = 5;
% gamma0 = 0*pi/180;
% theta0 = atan(0.5/20);

cx = 0;
cy = 0;
cz = 4.4;%3.28084;
gamma0 = 0*pi/180;
theta0 = atan(1/(0.5*1609.34));%atan(0.5/20);

%-------------------------
% Get Xvis Yvis Matrices
%-------------------------

rHorizon = ceil((m-1)/(2*alpha)*(alpha-theta0)+1);
% In actuality, the data is so compressed by that point that it's almost
% useless, so move rHorizon down a few rows to where we still have some hope
% of resolving the lane markers.
rHorizon = rHorizon + 10;

% If we take from that row down in the cropped image, that leaves us with:
mCropped = m-rHorizon+1; % rows in the cropped image.

Xvis = zeros(mCropped,n);
Yvis = zeros(size(Xvis));
Icropped = Ic(rHorizon:m,:,:);

for r = 1:mCropped
    
    rOrig = r + rHorizon - 1;
    u = rOrig - 1;
    
    for c = 1:n
        v = c - 1;
        cotStuff = cot(theta0-alpha+u*(2*alpha)/(m-1));
        otherArg = gamma0-alpha+v*(2*alpha)/(n-1);
        Xvis(r,c) = cz*cotStuff*sin(otherArg) + cx;
        Yvis(r,c) = cz*cotStuff*cos(otherArg) + cy;
    end
end

% To see how the image points spread out in the world frame, we can do:
% plot(Xvis(:), Yvis(:), 'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 4);

% We can also get a pretty good idea of what the image will turn out like
% by plotting these points as a surface and using the image as the color
% data for the faces.
figure('Units','normalized', 'Position', [0.05 0.05 0.7 0.4]);
% surf(Yvis, -Xvis, zeros(size(Xvis)), 'FaceColor', 'interp', ...
%     'CData', floor(Icropped*255), 'EdgeColor', 'none');
surf(Yvis, -Xvis, zeros(size(Xvis)), 'FaceColor', 'interp', ...
    'CData', floor(rgb2gray(Icropped)*255), 'EdgeColor', 'none');
view(2);
axis image;
colormap(gray);
set(gca, 'YLim', [-40 40]);
title('Bertozzi and Broggi''s Inverspse Perspective Mapping Results', 'FontSize', 10);
xlabel('Forward Distance (feet)');
ylabel('Side to Side Distance (feet)');

% For comparison, also plot where the lane markers and stop line should be
% according to the way they were generated.  (Remember that this method is
% supposed to take care of the offset between the camera and the origin.)
width = 1.5;
hOut = line([Yvis(end,1), Yvis(1,1)], [-9.84252, -9.84252], [0 0], 'LineStyle', '--', ...
    'Color', 'r', 'LineWidth', width);
line([Yvis(end,1), Yvis(1,1)], [9.84252, 9.84252], [0 0], 'LineStyle', '--', ...
    'Color', 'r', 'LineWidth', width);
hCent = line([Yvis(end,1), Yvis(1,1)], [0, 0], [0 0], 'LineStyle', '--', ...
    'Color', 'b', 'LineWidth', width);
hStop = line([76, 76], [-9.84252, 0], [0 0], 'LineStyle', '--', ...
    'Color', 'y', 'LineWidth', width);
legend([hOut, hCent, hStop], {'Ideal Outside', 'Ideal Center', 'Ideal Stop Line'}, ...
    'Location', 'NEO');

% The lane markers and stop line in the mapped image don't come out that
% close using this method.  The distance to the lane marker is particularly
% erroneous.



%---------------------------
%% Set up the xg, yg ranges
%---------------------------

% Based on the above plot command, the following ranges seem reasonable.
step = 0.125;
xg = 34:-step:-34; % Need to go top to bottom so the image won't get
                   % mirrored vertically.
yg = (10:step:200)';

%------------------------------------------------
%% Interpolate to get the world coordinate image
%------------------------------------------------

% Iw = griddata(Xvis, Yvis, Icropped, xg, yg, 'linear');
clear Iw
for d = 1:3
    Iw(:,:,d) = griddata(Xvis, Yvis, Ic(rHorizon:m,:,d), xg, yg, 'linear');
end
% Rotate it 90 degrees so the aspect ratio will let it fill the screen
% better.
Iw = rot90(Iw);
figure;
imagesc(Iw,[0 1]); colormap(gray);
axis image;
title('Bertozzi and Broggi''s Mapping Results');
xlabel('Column');
ylabel('Row');