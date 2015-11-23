function test_ManhattanWorld_2
    clc
    clear all
    close all

    % Read in an image
    im = imread('Test Images\pic001.jpg');
%     im = imread('Test Images\pic003.png');
    movegui(figure, 'northwest')
    imagesc(im) 
    im = rgb2gray(im);
    im = double(im);
    imsize = size(im);

    % Take the gradiant of the image
    gau = conv2([-1 0 1], [-1 0 1]');
    imedge = imfilter(im,gau);
    imedgex = imfilter(im, [-1 0 1], 'replicate');
    imedgey = imfilter(im, [-1 0 1]', 'replicate');
    magGradient = hypot(imedgex, imedgey);
    angGradient = atan2(imedgey, imedgex);

    figure
    imshow(imedge, [min(imedge(:)) max(imedge(:))])
    figure
    maxgrad = max(magGradient(:));
    if maxgrad > 0
        magGradient = magGradient/maxgrad;
    end
    imshow(magGradient, [0 1])
    drawnow
    tempAngGradient = wrapToPi(angGradient+pi/2);
    tempAngGradient(tempAngGradient<0) = tempAngGradient(tempAngGradient<0) + 2*pi;
%     figure
%     imshow(tempAngGradient*180/pi, [0 180])
    figure(2), hold on
    angGradient = tempAngGradient;
    [xx yy] = meshgrid(1:imsize(2), 1:imsize(1));
    quiver(xx, yy, cos(angGradient), sin(angGradient));
    
    alpha = linspace(-45,45,23)*pi/180;
    beta  = 0;%linspace(-45,45,41)*pi/180;
    gamma = 0;%linspace(-45,45,9)*pi/180;%0;%linspace(-4,4,3)*pi/180;
    
    
    % Compute P(E_u | m_u, Psi, u)P(m_u) for m_u = 1:5
    % P(m_u) = [0.02 0.02 0.02 0.04 0.9]
    % P(E_u | m_u, Psi, u) = P(E | m_u)P(Phi | m_u, Psi, u)
    %   where:
    %       P(E|m_u) = P_off(E) for m_u = 5, P_on(E) for m_u ~=5
    %       P(Phi|m_u,Psi,u) = P_ang(Phi-theta) for m_u = 1:3
    %                     or = Unifrom(Phi)
    
    epsilon = 0.1;
    tau = 6*pi/180;
    f_const = 797;
    
    p_off = [80 275 170 107 75 58 46 30 28 26 24 22 21 19 15 12 12 7 6 28]/1000;
    cdf_off = cumsum(p_off);
    cdf_off = cdf_off/max(cdf_off(:));
    p_on = [2 52 51 60 103 85 80 83 60 55 52 47 25 27 27 23 30 15 12 83]/1000;
    cdf_on = cumsum(p_on);
    cdf_on = cdf_on/max(cdf_on(:));
    m_u = [0.02 0.02 0.02 0.04 0.9];
    MAP = struct('likelihood', -inf(imsize), 'pixelLikelihood', zeros([imsize 5]), 'angles', zeros([imsize 3]));
    
    numbins = 64;
%     [counts, x] = imhist(magGradient, numbins);
%     y = cumsum(counts);
%     y = y/max(y);
%     y(diff(y)==0) = [];
%     P_e = interp1(y, linspace(1/length(y),1,length(y)), magGradient, 'neareast', 'extrap');
    maxMag = max(magGradient(:));
    P_onedge = interp1(cdf_on*maxMag, linspace(1/length(cdf_on),1,length(cdf_on)), magGradient, 'neareast', 'extrap');
    P_offedge = interp1(cdf_off*maxMag, linspace(1/length(cdf_off),1,length(cdf_on)), magGradient, 'neareast', 'extrap');
    P_e = repmat(P_onedge, [1 1 5]);
    P_e(:,:,5) = P_offedge;
%     P_e = bsxfun(@rdivide, P_e, sqrt(sum(P_e.^2, 3)));
    [pixelPos(:,:,1), pixelPos(:,:,2)] = meshgrid((1:imsize(2))-imsize(2)/2,(1:imsize(1))-imsize(1)/2);
          
    for a = alpha
%         fprintf('Alpha: %f\n', a*180/pi)
        for b = beta
%             fprintf('\tBeta: %f\n', b*180/pi)
            for c = gamma
%                 fprintf('\t\tGamma: %f\n', c*180/pi)
                P_ang = 1/(2*pi)*ones([imsize 5]);
                
                abc = computeR(a, b, c)*eye(3);
%                 abc(3,abc(3,:)==0) = 1;
                
                % Compute the vanishing points according to these
                % parameters and compare the image gradients to them.
                thetax = atan2(-(pixelPos(:,:,1)*abc(1,3) + f_const*abc(1,1)), pixelPos(:,:,2)*abc(1,3) + f_const*abc(1,2));
                thetay = atan2(-(pixelPos(:,:,1)*abc(2,3) + f_const*abc(2,1)), pixelPos(:,:,2)*abc(2,3) + f_const*abc(2,2));
                thetaz = atan2(-(pixelPos(:,:,1)*abc(3,3) + f_const*abc(3,1)), pixelPos(:,:,2)*abc(3,3) + f_const*abc(3,2));
                
                P_ang(:,:,1) = angDiff(angGradient, thetax, epsilon, tau);
                P_ang(:,:,2) = angDiff(angGradient, thetay, epsilon, tau);
                P_ang(:,:,3) = angDiff(angGradient, thetaz, epsilon, tau);  
                P_ang(:,:,4:5) = 1/(2*pi);
                
%                 modelLikelihood = bsxfun(@times, P_e, P_ang);
                modelLikelihood = P_e.*P_ang;

                pixelLikelihood = bsxfun(@times, modelLikelihood, reshape(m_u, [1 1 numel(m_u)]));
%                 pixelLikelihood = bsxfun(@times, P_ang, reshape(m_u, [1 1 numel(m_u)]));


                likelihood = sum(pixelLikelihood, 3);
                
                % Check if the new likelihood is higher, save the
                % parameters
                inds = likelihood > MAP.likelihood; 
                MAP.likelihood(inds) = likelihood(inds);
                currparamset = [a b c];
                for p = 1:5
                    % Save each pixels likelihood for each model
                    temp = MAP.pixelLikelihood(:,:,p);
                    pltemp = pixelLikelihood(:,:,p);
                    temp(inds) = pltemp(inds);
                    MAP.pixelLikelihood(:,:,p) = temp;
                end
                for p = 1:3
                    % Save the parameters that create this likelihood
                    temp = MAP.angles(:,:,p);
                    temp(inds) = currparamset(p);
                    MAP.angles(:,:,p) = temp;
                end
                drawnow; % To make the loop interruptible
            end
        end
    end
    assignin('base', 'MAP', MAP);

    
    [~, m_ind] = max(MAP.pixelLikelihood, [], 3);
    % Plot the classification of the edges
    colors = {'r' 'g' 'b' 'k' 'm'};
    figure, 
    for d = 1:5
        hold on
        [px py] = find(m_ind==d);
        plot(py, px, [colors{d} '.'], 'markersize', 5);
    end
    set(gca, 'ydir', 'reverse')

    % Break the image into a much coarser grid and find the orientation
    % according to that sector
    gridsize = 5;
    xstartind = round(linspace(1, imsize(2), gridsize));
    ystartind = round(linspace(1, imsize(1), gridsize));
    
    figure(1), hold on
    for xind = 1:length(xstartind)-1
        for yind = 1:length(ystartind)-1
            xindex = ystartind(xind):ystartind(xind+1);
            yindex = xstartind(yind):xstartind(yind+1);
            temp_mind = m_ind(xindex, yindex);
            
            xpos = (xstartind(xind)+xstartind(xind+1))/2;
            ypos = (ystartind(yind)+ystartind(yind+1))/2;
            
            figure(1)
            ang = zeros(1,3);
            for p = 1:3
                temp = MAP.angles(:,:,p);
                temp = temp(xindex, yindex);
                temp = temp(temp_mind==p);
                ang(p) = mode(temp(:)); 
%                 [px, py] = find(temp_mind==p);
%                 quiver(px+xstartind(xind)-1, py+ystartind(yind)-1, cos(temp), sin(temp), colors{p})
                fprintf('Percentage of pixels for %g: %3.2f%%\n', p, sum(temp_mind(:)==p)/numel(temp_mind)*100);
            end
            ang(isnan(ang)) = 0;

%             loc = [xpos ypos] - imsize/2; ang = [atan2(loc(2), loc(1)) atan2(loc(1), loc(2)) 0];
            R = computeR(ang(1), ang(2), ang(3))*eye(3);
            
            thetax = atan2(-(pixelPos(:,:,1)*abc(1,3) + f_const*abc(1,1)), pixelPos(:,:,2)*abc(1,3) + f_const*abc(1,2));
            thetay = atan2(-(pixelPos(:,:,1)*abc(2,3) + f_const*abc(2,1)), pixelPos(:,:,2)*abc(2,3) + f_const*abc(2,2));
            thetaz = atan2(-(pixelPos(:,:,1)*abc(3,3) + f_const*abc(3,1)), pixelPos(:,:,2)*abc(3,3) + f_const*abc(3,2));
            quiver(xpos, ypos, 10*R(1,1), 10*R(2,1), 'color', 'r', 'linewidth', 3);
            quiver(xpos, ypos, 10*R(1,2), 10*R(2,2), 'color', 'g', 'linewidth', 3);
            quiver(xpos, ypos, 10*R(1,3), 10*R(2,3), 'color', 'b', 'linewidth', 3);
%             quiver(xpos, ypos, 10*R(1,1), 10*R(2,1), 'color', 'r', 'linewidth', 3);
%             quiver(xpos, ypos, 10*R(1,2), 10*R(2,2), 'color', 'g', 'linewidth', 3);
%             quiver(xpos, ypos, 10*R(1,3), 10*R(2,3), 'color', 'b', 'linewidth', 3);
%             fprintf('Angles: %s\n', mat2str(ang*180/pi, 3))
        end
    end
end

function z = angDiff(x, y, epsilon, tau)
    z = zeros(size(x));
    diffang = abs(wrapToPi(x - y));
    ind = diffang <= tau;
%     z(ind) = exp(-diffang(ind));
    diffang = abs(wrapToPi(x - y - pi));
    ind = ind | diffang <= tau;
%     ind = diffang <= tau;
%     z(ind) = exp(-diffang(ind));
    z = epsilon/(2*pi-4*tau)*ones(size(x));
    z(ind) = (1-epsilon)/(4*tau);

end

function R = computeR(a, b, c)

    R = [cos(a)*cos(b) -sin(a)*cos(c) + cos(a)*sin(b)*sin(c) sin(a)*sin(c)+cos(a)*sin(b)*cos(c);
         sin(a)*cos(b) cos(a)*cos(c) + sin(a)*sin(b)*sin(c) -cos(a)*sin(c)+sin(a)*sin(b)*cos(c);
         -sin(b)                -cos(b)*sin(c)                  cos(b)*cos(c)];

end

function y = wrapToPi(x)
    % Makes x between [-pi pi]
    y = mod(x, sign(x)*2*pi);
    y(abs(y)>pi) = y(abs(y)>pi) - sign(y(abs(y)>pi))*2*pi;
end

