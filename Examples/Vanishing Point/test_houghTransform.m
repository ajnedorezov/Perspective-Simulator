function test_houghTransform
    clc
    clear all
    close all
     
    im_orig = imread('Test Images\pic001.jpg');
    movegui(figure, 'northwest')
    imagesc(im_orig)
    title('Original Image')
    
    % Convert the image to grayscale
    im = double(rgb2gray(im_orig));
    imsize = size(im);
        
    % Take the gradiant of the image
    imdx = imfilter(im, [-1 0 1], 'replicate');
    imdy = imfilter(im, [-1 0 1]', 'replicate');
    
    % Compute the magnitude of the gradient
    magGradient = hypot(imdx, imdy);
    figure, imshow(magGradient, [0 max(magGradient(:))])
    title('Magnitude of the Gradient')
    
    % Compute the angle of the gradient wrap values to be between [0, pi]
    angGradient = atan2(imdy, imdx);
    angGradient(angGradient<0) = angGradient(angGradient<0) + 2*pi;
    figure, imshow(angGradient*180/pi, [-180 180]), colorbar, hold on
    [xx yy] = meshgrid(1:imsize(2), 1:imsize(1));
    quiver(xx, yy, sin(angGradient), cos(angGradient));
    title({'Angle of the Gradient' 'with overlaid quivers showing gradient direction'})
    
    % Compute the P_edge probabilities from the gradients magnitude
    P_edge = computePOnOff(magGradient);
%     figure, imagesc(P_edge.on), colorbar
    figure, imagesc(P_edge.on >= .75), colorbar
    title('Probability pixel is an edge pixel')
    
    % Compute the Hough Transform
    HT = computeHoughTransform(P_edge.on, 0.75, 5);
    figure
    imshow(imadjust(mat2gray(HT.accumulator)), ...
        'XData', HT.theta_step*180/pi,...
        'YData', HT.r_step,...
        'InitialMagnification', 'fit');
    title('Hough Tranform'), ylabel('\rho'), xlabel('\theta (deg)'),
    axis on, axis normal, colormap(hot)
    hold on, plot(HT.lines.theta*180/pi, HT.lines.r, 'o', 'markerfacecolor', 'auto')
    
    % Draw the Hough line on the original image
    figure(4), hold on
    largenum = 10000;
    for n = 1:length(HT.lines.centerpoint)
        x = [-largenum 0 largenum]*cos(HT.lines.theta(n) + pi/2) + HT.lines.centerpoint(1,n);
        y = [-largenum 0 largenum]*sin(HT.lines.theta(n) + pi/2) + HT.lines.centerpoint(2,n);
        
        plot(y, x, '-o', 'xliminclude', 'off', 'yliminclude', 'off')        
    end
    
    assignin('base', 'HT', HT)
    
end

function P_edge = computePOnOff(magGradient)
    % Obtain a histogram of the gradient magnitude
    [counts, bins] = hist(magGradient(:), 20);
    
    % Compute the cdf of the histogram
    cdfOnEdge = cumsum(counts);
    cdfOnEdge = (cdfOnEdge - min(cdfOnEdge(:)))/(max(cdfOnEdge(:)) - min(cdfOnEdge(:)));
    P_edge.on = interp1(bins, cdfOnEdge, magGradient, 'nearest', 'extrap');
    
    % Let P_offEdge be directly related to P_onEdge
    P_edge.off = 1 - P_edge.on;
end

function HT = computeHoughTransform(P_edge, threshold, numlines)
    % The Hough transform searches for the best set of parameters 
    % (r, theata) that goups the lines detected by P_edge. Using the
    % equaiton for a line:
    %   x*cos(theata) + y*sin(theata) = r
    
    % Set Hough Transform Parameters
    imsize = size(P_edge);
    pixel_neighborhood = [15 5];
    
    % Get a logical representation of the edges
    edges = P_edge > threshold;
    
    % Initialize the parameter search
    imcenter = imsize/2;
    r_max = hypot(imsize(1), imsize(2));
    num_r_steps = 100;
    r_step = 2*r_max/num_r_steps;
    r_spacing = linspace(-r_max, r_max, num_r_steps);
    theta_step = linspace(-90,89,180)*pi/180;
    
    % For each edge pixel try each theata angle and increment the
    % Accumulator cell corresponding the given theta and resulting r
    ind = find(edges);
    [x y] = ind2sub(imsize, ind);
    x = x - imcenter(2);
    y = y - imcenter(1);
    
    % Compute the r for the given edge pixels
    r = bsxfun(@times, x(:), cos(theta_step(:))') + ...
        bsxfun(@times, y(:), sin(theta_step(:))');
    
    % Find the index r belong to each theata
    r_index = num_r_steps/2 + round(r/r_step);
    
    % Fill in the accumulator    
    accumulator = hist(r, r_spacing);
    
    % Find n lines that represent the best fit
    gradient_x = imfilter(accumulator, [-1 0 1], 'circular');
    gradient_y = imfilter(accumulator, [-1 0 1]', 'circular');
    gradient_mag = hypot(gradient_x, gradient_y);
    gradient_ang = atan2(gradient_y, gradient_x);
    hough_edgemap = 255*gradient_mag/max(gradient_mag(:));
    lines = struct('r', zeros(1, numlines), 'theta', zeros(1, numlines),...
                   'centerpoint', zeros(2, numlines));
    temp_hough_edgemap = hough_edgemap;
    mapsize = size(temp_hough_edgemap);
    peaks = zeros(2, numlines);
    for n = 1:numlines
        % Find the maximum value in the edgemap
        [val ind] = max(temp_hough_edgemap(:));                 %#ok<ASGLU>
        [u v] = ind2sub(mapsize, ind);
        peaks(:,n) = [u; v];
        
        % Store these line parameters
        lines.r(n) = r_spacing(u);
        lines.theta(n) = theta_step(v);
        lines.centerpoint(:,n) = lines.r(n)*[cos(lines.theta(n)); sin(lines.theta(n))] + imcenter';
        
        % Suppress the neighbor hood of the parameter just found, have to
        % account for the odd symmetry of the sin function
        [uu, vv] = meshgrid((u - pixel_neighborhood(1)):(u + pixel_neighborhood(1)), ...
                            max(v - pixel_neighborhood(2), 1):min(v + pixel_neighborhood(2), mapsize(2)));
        flipind = uu <= 0;
        uu(flipind) = imsize(2) + uu(flipind);
        vv(flipind) = imsize(1) - vv(flipind) + 1;
        flipind = vv > mapsize(2);
        uu(flipind) = uu(flipind) - mapsize(2);
        vv(flipind) = mapsize(1) - vv(flipind) + 1;
        uu = uu(:);
        vv = vv(:);
        
        temp_hough_edgemap(sub2ind(mapsize, uu, vv)) = 0;
        set(figure(99), 'WindowStyle','docked'), imagesc(temp_hough_edgemap), hold on, plot(peaks(2,:), peaks(1,:),'ko')

        
        
    end
    
    % Log additional variables that may be useful
    HT.r_step = r_spacing;
    HT.theta_step = theta_step;
    HT.accumulator = accumulator;
    HT.gradient_mag = gradient_mag;
    HT.gradient_ang = gradient_ang;
    HT.hough_edgemap = hough_edgemap;
    HT.lines = lines;
    
end


function y = wrapToPi(x)
    % Wrap the angle values to be between [-pi, pi]
    y = mod(x, sign(x)*2*pi);
    y(abs(y)>pi) = y(abs(y)>pi) - sign(y(abs(y)>pi))*2*pi;
end
    
    