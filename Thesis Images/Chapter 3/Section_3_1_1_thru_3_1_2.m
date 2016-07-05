% Script to make images for Section 3.1.1 - 3.1.2 of thesis

%% Load in the image
im = imread('Thesis Images\Chapter 3\BaseImageSection_3_1_1.png');

figure, imshow(im)
title('Original Camera Image')

% saveIm = getframe(gcf);
% imwrite(saveIm.cdata, 'Thesis Images\Chapter 3\figure_3_1-OriginalImage.png');
saveas(gcf, 'Thesis Images\Chapter 3\figure_3_1-OriginalImage.png', 'png');

%% Run the Canny edge detector on grayscale version of original image
imsize = size(im);
myVPTracker = VPTracker(1000, imsize(1:2));
% myVPTracker.Update(im, [0 0]);
greyim = double(rgb2gray(im));
edges.cdata = edge(greyim, 'canny'); % More lines
edges.colomap = [];

figure, imshow(edges.cdata)
title('Canny Edge Detections');

% saveIm = getframe(gcf);
% imwrite(saveIm.cdata, 'Thesis Images\Chapter 3\figure_3_2-CannyEdgeDetections.png');
saveas(gcf, 'Thesis Images\Chapter 3\figure_3_2-CannyEdgeDetections.png', 'png');

%% Perform the Hough Transform

[H,T,R] = hough(edges.cdata);

% Find the peaks in the Hough transform accumulator matrix
% corresponding to the line estimates
P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));

% Find the equations fo the lines
lines = houghlines(edges.cdata,T,R,P,'FillGap',.25*min(size(edges.cdata)),'MinLength',7);
for k = 1:length(lines)
    xy = [lines(k).point1; lines(k).point2];
    xydiff = diff(xy, 1);
    lines(k).plottheta = atan2(xydiff(2), xydiff(1));
end

figure, imshow(H,[],'XData',T,'YData',R,'InitialMagnification','fit');
title('Hough Transform of Edge Pixels')
hold on
plot(T(P(:,2)),R(P(:,1)),'s','color','m', 'MarkerSize', 8, 'LineWidth', 3.5);
xlabel('\theta'), ylabel('\rho');
axis on, axis normal; colorbar

% saveIm = getframe(gcf);
% imwrite(saveIm.cdata, 'Thesis Images\Chapter 3\figure_3_3-HoughTransformed.png');
saveas(gcf, 'Thesis Images\Chapter 3\figure_3_3-HoughTransformed.png', 'png');

%% Draw the detected lines on the original image

figure, imshow(im)
title('Detected Lines from Hough Transform')
hold on

% Draw the Hough lines
for n = 1:length(lines)
    x = lines(n).point1(1) + cos(lines(n).plottheta)*[-10000 0 10000];
    y = lines(n).point1(2) + sin(lines(n).plottheta)*[-10000 0 10000];

    plot(x, y, 'm', 'xliminclude', 'off', 'yliminclude', 'off', 'LineWidth', 4)
end


% saveIm = getframe(gcf);
% imwrite(saveIm.cdata, 'Thesis Images\Chapter 3\figure_3_4-DetectedLines.png');
saveas(gcf, 'Thesis Images\Chapter 3\figure_3_4-DetectedLines.png', 'png');

%% Add the intercept points as well

figure, imshow(im)
title('Detected Lines from Hough Transform')
hold on

% Draw the Hough lines
for n = 1:length(lines)
    x = lines(n).point1(1) + cos(lines(n).plottheta)*[-10000 0 10000];
    y = lines(n).point1(2) + sin(lines(n).plottheta)*[-10000 0 10000];

    plot(x, y, 'm', 'xliminclude', 'off', 'yliminclude', 'off', 'LineWidth', 2)
end

% Create the A and b matrix
numLines = length(lines);
A = zeros(numLines, 2);
b = zeros(numLines, 1);
toDelete = [];
for l = 1:numLines
    A(l, :) = [-tan(lines(l).plottheta) 1];
    b(l) = A(l, :)*lines(l).point1(:);
    if isnan(A(l,1))
        toDelete(end+1) = l;
    end
end
A(toDelete, :) = [];
b(toDelete) = [];
if isempty(A)
    % No lines were computed this frame.
    return
end
pts = myVPTracker.findVert(A, b);

hold on
plot(pts(1,:), pts(2,:), 'cx', 'markersize', 10, 'linewidth', 4)
title('Candidate Vanishing Point using Line Intersections')

% saveIm = getframe(gcf);
% imwrite(saveIm.cdata, 'Thesis Images\Chapter 3\figure_3_5-CandidateVPs.png');
saveas(gcf, 'Thesis Images\Chapter 3\figure_3_5-CandidateVPs.png', 'png');
