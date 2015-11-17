function test_manhattan
    clc
    clear all
    close all
     
    % Read in an image
    im_orig = imread('Test Images\pic002.jpg');
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
    angGradient(angGradient>=pi) = angGradient(angGradient>=pi) - pi;
%     angGradient(angGradient<0) = wrapToPi(angGradient(angGradient<0) + 2*pi);
    figure, imshow(angGradient*180/pi, [-180 180]), colorbar, hold on
    [xx yy] = meshgrid(1:imsize(2), 1:imsize(1));
    quiver(xx, yy, magGradient.*sin(angGradient), magGradient.*cos(angGradient));
    title({'Angle of the Gradient' 'with overlaid quivers showing gradient direction'})
    
    % Compute the P_edge probabilities from the gradients magnitude
    P_edge = computePOnOff(magGradient);
    figure, imagesc(P_edge.on), colorbar
    title('Probability pixel is an edge pixel')
%     figure, imagesc(P_edge.off), colorbar
%     title('Probability pixel is NOT an edge pixel')
    
    % Search through angles to find the Manhattan World (MW) orientations
    tic
    MW = computeManhattanWorldModel(P_edge, angGradient);
    fprintf('Time to compute MW angles: %f seconds\n', toc)
    
    % Create a new picture depicting the labeling of the edges
    figure, hold on
    colors = {'r' 'g' 'b' 'y' 'k'};
    for d = 1:5
        ind = MW.edge_labels == d;
        plot(xx(ind), yy(ind), [colors{d} '.']);
%         quiver(xx(ind), yy(ind), magGradient(ind).*sin(angGradient(ind)), magGradient(ind).*cos(angGradient(ind)), colors{d});
        fprintf('Percentage of pixels of edge type %d: %0.2f%%\n', d, sum(ind(:))/numel(MW.edge_labels)*100)
    end
    set(gca, 'ydir', 'reverse')
    title({'Classification of pixels' '1 - Red, 2 - Green, 3 - Blue' '4 - Yellow, 5 - Black'})
    
    % Split the image into several regions and plot the three MW vectors
    % over top of the original image
    movegui(figure, 'northwest'), imagesc(im_orig), hold on
    title('MW Orientation Regional Estimates')
    
    gridsize = 7;
    xstartind = round(linspace(1, imsize(2), gridsize));
    ystartind = round(linspace(1, imsize(1), gridsize));
    focallength = 797;
    quivlength = 10;
    
    for xind = 1:length(xstartind)-1
        for yind = 1:length(ystartind)-1
            
            xpos = (xstartind(xind)+xstartind(xind+1))/2;
            ypos = (ystartind(yind)+ystartind(yind+1))/2;
            
            MWangles = MW.angles;
            
            abc = computeR(MWangles(1), MWangles(2), MWangles(3));%*eye(3);
            thetax = atan2(((xpos - imsize(2)/2)*abc(1,3) + focallength*abc(1,1)), ((ypos - imsize(1)/2)*abc(1,3) + focallength*abc(1,2)));
            thetay = atan2(((xpos - imsize(2)/2)*abc(2,3) + focallength*abc(2,1)), ((ypos - imsize(1)/2)*abc(2,3) + focallength*abc(2,2)));
            thetaz = atan2(((xpos - imsize(2)/2)*abc(3,3) + focallength*abc(3,1)), ((ypos - imsize(1)/2)*abc(3,3) + focallength*abc(3,2)));          
            quiver(xpos, ypos, 10*sin(thetax), 10*cos(thetax), 'color', 'r', 'linewidth', 3);
            quiver(xpos, ypos, 10*sin(thetay), 10*cos(thetay), 'color', 'g', 'linewidth', 3);
            quiver(xpos, ypos, 10*sin(thetaz), 10*cos(thetaz), 'color', 'b', 'linewidth', 3);

%             quiver3(xpos,ypos,10,quivlength*abc(1,1),quivlength*abc(1,2),quivlength*abc(1,3), 'r', 'linewidth', 3)
%             quiver3(xpos,ypos,10,quivlength*abc(2,1),quivlength*abc(2,2),quivlength*abc(2,3), 'g', 'linewidth', 3)
%             quiver3(xpos,ypos,10,quivlength*abc(3,1),quivlength*abc(3,2),quivlength*abc(3,3), 'b', 'linewidth', 3)

        end
    end
    
    % Write data out to the base workspace for post analysis
    assignin('base', 'magGradient', magGradient)
    assignin('base', 'angGradient', angGradient)
    assignin('base', 'MW', MW)
    assignin('base', 'P_edge', P_edge)
    
    drawnow
end

function P_edge = computePOnOff(magGradient)
    % Obtain a histogram of the gradient magnitude
    [counts, bins] = hist(magGradient(:), 20);
    
    % Compute the cdf of the histogram
    cdfOnEdge = cumsum(counts);
    cdfOnEdge = (cdfOnEdge - min(cdfOnEdge(:))) / ...
                (max(cdfOnEdge(:)) - min(cdfOnEdge(:)));
    P_edge.on = interp1(bins, cdfOnEdge, magGradient, 'nearest', 'extrap');
    
    % Let P_offEdge be directly related to P_onEdge
    P_edge.off = 1 - P_edge.on;
end

function MW = computeManhattanWorldModel(P_edge, angGradient)
    % This function does a parameter search for the rotation angles that
    % best describe the vanishing points of the image. This essentially
    % classifies each pixel as either an edge belonging to an x,y,z axis,
    % an edge belonging to a random axis and not an edge. This
    % classification is performed using a mixture model of image gradients
    % and gradient angles relative to the current rotation guess.

    % Gather information about the image
    imsize = size(angGradient);
    [pixelPos(:,:,1) pixelPos(:,:,2)] = ...
        meshgrid((1:imsize(2))-imsize(2)/2, (1:imsize(1))-imsize(1)/2);
    
    % Set the parameters of the search
    % ---------------------------------
    % A best guess at a focal length
    focallength = 797;
    epsilon = 0.1;
    % Angular error gate 
    tau = 4*pi/180;
    % Prior probabilites
    P_m = [0.02 0.02 0.02 0.04 0.9];
    P_m = reshape(P_m, [1 1 length(P_m)]);
    uniformP_ang = 1/(2*pi);
    % Compass angle
    alpha = linspace(-45, 45, 9)*pi/180;
    % Elevation angle
    beta = linspace(-40, 40, 9)*pi/180;
    % Twist angle
    gamma = linspace(-4, 4, 5)*pi/180;
    
    % Initialize the data sturctures to hold the results
    MW.model_likelihood = -inf;
    MW.pixel_likelihood = zeros([imsize 5]);
    MW.angles = zeros(1,3);
    
    % Vary the compass angle alpha
    for a = alpha
        % Vary the elevation angle beta
        for b = beta
            % Vary the twist angle 
            for c = gamma
%                 fprintf('Testing: alpha = %0.3f, beta = %0.3f, gamma = %0.3f\n', a*180/pi, b*180/pi, c*180/pi);

                abc = computeR(a, b, c)*eye(3);
                
                thetax = atan2(...
                    -(pixelPos(:,:,1)*abc(1,3) + focallength*abc(1,1)),...
                      pixelPos(:,:,2)*abc(1,3) + focallength*abc(1,2));
                thetay = atan2(...
                    -(pixelPos(:,:,1)*abc(2,3) + focallength*abc(2,1)),...
                      pixelPos(:,:,2)*abc(2,3) + focallength*abc(2,2));
                thetaz = atan2(...
                    -(pixelPos(:,:,1)*abc(3,3) + focallength*abc(3,1)),...
                      pixelPos(:,:,2)*abc(3,3) + focallength*abc(3,2));
                
                P_ang_alpha = computePang(angGradient, thetax, epsilon, tau);
                P_ang_beta = computePang(angGradient, thetay, epsilon, tau);
                P_ang_gamma = computePang(angGradient, thetaz, epsilon, tau);
                

                % Compute the pixel_likelihood for the image
                new_pixel_likelihood = zeros([imsize 5]);
                new_pixel_likelihood(:,:,1) = P_edge.on .* P_ang_alpha;
                new_pixel_likelihood(:,:,2) = P_edge.on .* P_ang_beta;
                new_pixel_likelihood(:,:,3) = P_edge.on .* P_ang_gamma;
                new_pixel_likelihood(:,:,4) = P_edge.on .* uniformP_ang;
                new_pixel_likelihood(:,:,5) = P_edge.off .* uniformP_ang;
                        
                % Compute the pixel's model likelihood
                new_likelihood = log10(sum(bsxfun(@times, new_pixel_likelihood, P_m), 3));
                new_likelihood = sum(new_likelihood(:));
                        
                % See if the new MW angles are a better fit
                if new_likelihood > MW.model_likelihood
                    MW.model_likelihood = new_likelihood;
                    MW.pixel_likelihood = new_pixel_likelihood;
                    MW.angles = [a b c];
                end

            end
        end
    end
    
    % Grab the labeling for each pixel using the pixel likelihood from each
    % edge model in the above mixture model.
    [MW.best_pixel_ikelihood, MW.edge_labels] = max(MW.pixel_likelihood, [], 3);
    
    fprintf('Selected angles:\n\tAlpha: %f (deg)\n\tBeta: %f (deg)\n\tGammma: %f (deg)\n', MW.angles(1)*180/pi, MW.angles(2)*180/pi, MW.angles(3)*180/pi);
    
end

function R = computeR(a, b, c)
    % Computes the rotation matrix that rotates from the xyz axis to the
    % camera axis.
    
    R = [cos(a)*cos(b) -sin(a)*cos(c) + cos(a)*sin(b)*sin(c) sin(a)*sin(c)+cos(a)*sin(b)*cos(c);
         sin(a)*cos(b) cos(a)*cos(c) + sin(a)*sin(b)*sin(c) -cos(a)*sin(c)+sin(a)*sin(b)*cos(c);
         -sin(b)                -cos(b)*sin(c)                  cos(b)*cos(c)];

end

function z = computePang(x, y, epsilon, tau)
%     z = epsilon/(2*pi - 4*tau)*ones(size(x));
%     angularDiff = abs(wrapToPi(x - y));
%     diff1ind = angularDiff < tau;
%     angularDiffPi = abs(wrapToPi(x - y + pi));
%     diff2ind = angularDiffPi < tau;
%     z(diff1ind | diff2ind) = (1-epsilon)/(4*tau);
% %     z(diff2ind) = (1-epsilon)/(4*tau);

    % Changed the probability function of the original paper to be gaussian
    % as shown below. This does not seem to have effected anything.
    z = zeros(size(x));
    angularDiff = abs(wrapToPi(x - y));
    diffind = angularDiff < tau;
    z(diffind) = exp(-angularDiff(diffind));
    angularDiff = abs(wrapToPi(x - y - pi));
    diffind = angularDiff < tau;
    z(diffind) = exp(-angularDiff(diffind));
end

function y = wrapToPi(x)
    % Wrap the angle values to be between [-pi, pi]
    y = mod(x, sign(x)*2*pi);
    y(abs(y)>pi) = y(abs(y)>pi) - sign(y(abs(y)>pi))*2*pi;
end