function test_ComputeLines
    clc
    clear all
    close all

    % Read in an image
    im = imread('Test Images\pic001.jpg');
%     im = imread('Test Images\pic003.png');
    movegui(figure, 'northwest')
    imagesc(im)   
    
    % Perform edge detection
    greyim = double(rgb2gray(im));
    imedge = edge(greyim, 'sobel');
%     imedge = edge(greyim, 'canny');
    movegui(figure, 'southwest')
    imagesc(imedge);
    
    % Find the x & y derivatives
    xdir = imfilter(greyim, [-1 0 1]);
    ydir = imfilter(greyim, [-1 0 1]');
    immag = hypot(xdir, ydir);
    imang = atan2(ydir, xdir);
    imang = wrapToPi(imang);
    imang(imang<0) = imang(imang<0) + pi;
    
%     % Draw the direction of the x & y derivatives for each of the edges.
%     [xx, yy] = meshgrid(1:size(greyim,2), 1:size(greyim,1));
%     figure(2)
%     hold on
%     quiver(xx(imedge), yy(imedge), cos(imang(imedge)), sin(imang(imedge)), 'color', 'c')
    
    % Break the image into a much coarser grid and average the x & y
    % derivates within that grid space;
    gridsize = 3;
    [sx, sy] = size(greyim);
    xstartind = round(linspace(1, sy, gridsize));
    ystartind = round(linspace(1, sx, gridsize));
    
    % Compute the average gradient for each grid space
    figure(1), hold on
    sector = struct;
    for xind = 1:length(xstartind)-1
        for yind = 1:length(ystartind)-1
            magGradient = immag(ystartind(xind):ystartind(xind+1),...
                                xstartind(yind):xstartind(yind+1));
            angGradient = imang(ystartind(xind):ystartind(xind+1),...
                                xstartind(yind):xstartind(yind+1));
            edges = imedge(ystartind(xind):ystartind(xind+1),...
                           xstartind(yind):xstartind(yind+1));
            pixelPos = [];
            [pixelPos(:,:,1), pixelPos(:,:,2)] = meshgrid(...
                ((xstartind(yind):xstartind(yind+1)) + 1) - sx/2,...
                ((ystartind(xind):ystartind(xind+1)) + 1) - sy/2);
            sector(xind, yind).MAP = findManhattanAngles(magGradient, angGradient, pixelPos, edges);

            sector(xind, yind).xpos = (xstartind(xind)+xstartind(xind+1))/2;
            sector(xind, yind).ypos = (ystartind(yind)+ystartind(yind+1))/2;
            
            t = sector(xind, yind);
            a = mean(t.MAP.angles(:,:,1));
            b = mean(t.MAP.angles(:,:,2));
            c = mean(t.MAP.angles(:,:,3));
            R = computeR(a, b, c)*eye(3);
            figure(1)
            quiver(t.xpos, t.ypos, 10*R(1,1), 10*R(2,1), 'color', 'r', 'linewidth', 3);
            quiver(t.xpos, t.ypos, 10*R(1,2), 10*R(2,2), 'color', 'g', 'linewidth', 3);
            quiver(t.xpos, t.ypos, 10*R(1,3), 10*R(2,3), 'color', 'b', 'linewidth', 3);
%             fprintf('Angles: %s\n', mat2str(t.MAP.angles*180/pi, 3))
        end
    end
    
    assignin('base', 'sector', sector);
    % Plot color for the edges identifying the axis it belongs to, only
    % when grid size = 2
    figure(2), hold on
    for s = 1:numel(sector)
        [~, ind] = max(sector(s).MAP.pixelLikelihood,[],3);
        figure(2)
        inds = ind == 1;
        plot(sx/2+sector(s).MAP.pixelpos(inds,:,1), sy/2 + sector(s).MAP.pixelpos(inds,:,2),'r.')
        inds = ind == 2;
        plot(sx/2+sector(s).MAP.pixelpos(inds,:,1), sy/2 + sector(s).MAP.pixelpos(inds,:,2),'g.')
        inds = ind == 3;
        plot(sx/2+sector(s).MAP.pixelpos(inds,:,1), sy/2 + sector(s).MAP.pixelpos(inds,:,2),'y.')
    end
    set(gca, 'ydir', 'reverse')
    
    % Create the A and b matrix
    A = zeros(numel(sector), 2);
    b = zeros(numel(sector), 1);
    count = 1;
    for xind = 1:length(xstartind)-1
        for yind = 1: length(ystartind)-1
            t = sector(xind, yind);
%             A(count, :) = [-t.ymean t.xmean];
%             b(count) = t.xmean*A(count, :)*[t.xpos t.ypos]';
%             A(count, :) = [-cos(t.MAP.angles(1))/cos(t.MAP.angles(2)) 1];
%             b(count) = A(count, :)*[t.xpos t.ypos]';
            A(count, :) = [-tan(t.MAP.angles(1)) 1];
            b(count) = A(count, :)*[t.xpos t.ypos]';
            if isnan(A(count,1))
                A(count, 1) = 0;
                b(count) = 0;
            end
            count = count + 1;
        end
    end
    
    % Compute the intercept points
    pts = findVert(A, b);
    if isempty(pts)
        disp('No intersections found')
    else
        figure(1)
        plot(pts(1,:), pts(2,:), 'm.')
        plot(mean(pts(1,:)), mean(pts(2,:)), 'ms')
    end
    

end

function pts = findVert(A, b)
    pts = [];
    
    [numConstraints, numVariables] = size(A);
    
    % If num constraints > num variables
    if numConstraints > numVariables
        A = [A eye(numConstraints)];
        [numConstraints, numVariables] = size(A);
    end

    ind = nchoosek(1:numVariables, numConstraints);
    % Ignore combinations that don't use the first two variables
    temp = (ind == 1 | ind == 2);
    ind(~(temp(:,1) & temp(:,2)), :) = [];

    numCombinations = size(ind, 1);%nchoosek(n,m);
    for k = 1:numCombinations
        y = zeros(numVariables,1);
        % Solve the eqution using the current set of variables
        x = A(:,ind(k,:))\b;
        if all(isinf(x) | isnan(x))
            % Ingore parallel lines
            continue
        end
        
        % Store the values into the proper indicies
        y(ind(k,:)) = x;
        pts = [pts y];                                      %#ok<AGROW>
    end
end

function [MAP] = findManhattanAngles(magGradient, angGradient, pixelpos, edges)
    numelem = sum(edges(:));
    MAP = struct('likelihood', -inf(numelem,1), 'P_edges', -inf(numelem,1,5), 'angles', zeros(numelem,1,3), 'newppos', nan(numelem,2,5), 'pixelLikelihood', -inf(numelem,1,5));
    numbins = 64;
    
    maxG = max(magGradient(:));
    if maxG > 0
        magGradient = magGradient/maxG;
    end
    counts = imhist(magGradient, numbins);
    y = cumsum(counts);
    y = y/max(y);
    y(diff(y)==0) = [];
    P_edge = interp1(y, linspace(1/(length(y)+1),1,length(y)), magGradient, 'neareast', 'extrap');
    
    % Only use the edges
    P_edge = P_edge(edges);
    P_edge = repmat(P_edge, [1 1 5]);
    P_edge(:,:,5) = 1 - P_edge(:,:,5);
    temp = pixelpos(:,:,1);
    newppos(:,:,1) = temp(edges);
    temp = pixelpos(:,:,2);
    newppos(:,:,2) = temp(edges);
    angGradient = angGradient(edges);
    if isempty(P_edge)
        return
    end

    % Do a search over the angle parameters
    alpha = (linspace(-45,45,19))*pi/180;
    beta = (linspace(-45,45,19))*pi/180;
    gamma = (linspace(-4,4,9))*pi/180;
    
    MAP = AngleSearch(alpha, beta, gamma, newppos, P_edge, angGradient, MAP);
    
    MAP.pixelpos = newppos;
    
end

function MAP = AngleSearch(alpha, beta, gamma, pixelpos, P_edge, angGradient, MAP)
    m_u = [0.02 0.02 0.02 0.04 0.9];
    m_u = m_u/norm(m_u);
    m_u = reshape(m_u, [1 1 numel(m_u)]);
    epsilon = 0.1;
    tau = 6*pi/180;
    f_const = 797;
   
    for a = alpha
        for b = beta
            for c = gamma
                abc = computeR(a, b, c)*eye(3);
                
                thetax = atan2(-(pixelpos(:,:,1) + f_const*abc(1,1)), pixelpos(:,:,2) + f_const*abc(1,2));
                thetay = atan2(-(pixelpos(:,:,1) + f_const*abc(2,1)), pixelpos(:,:,2) + f_const*abc(2,2));
                thetaz = atan2(-(pixelpos(:,:,1) + f_const*abc(3,1)), pixelpos(:,:,2) + f_const*abc(3,2));
                
                P_psi(:,:,1) = angDiff(angGradient, thetax, epsilon, tau);
                P_psi(:,:,2) = angDiff(angGradient, thetay, epsilon, tau);
                P_psi(:,:,3) = angDiff(angGradient, thetaz, epsilon, tau);   
                P_psi(:,:,4:5) = 1/(2*pi);
                
                % Compute the liklihood
                P_ang = bsxfun(@times, P_edge, P_psi);
                P_ang = bsxfun(@times, P_ang, m_u);

                likelihood = sum(P_ang, 3);
                
                % Check if the new likelihood is higher, save the
                % parameters
                inds = likelihood > MAP.likelihood; 
                MAP.likelihood(inds) = likelihood(inds);
                currparamset = [a b c];
                for p = 1:3
                    % Save each pixels likelihood for each model
                    temp = MAP.pixelLikelihood(:,:,p);
                    pltemp = P_ang(:,:,p);
                    temp(inds) = pltemp(inds);
                    MAP.pixelLikelihood(:,:,p) = temp;
                    % Save the parameters that create this likelihood
                    temp = MAP.angles(:,:,p);
                    temp(inds) = currparamset(p);
                    MAP.angles(:,:,p) = temp;
                end
            end
        end
    end
end

function z = angDiff(x, y, epsilon, tau)
    diffang = abs(wrapToPi(x - y));
    ind = diffang <= tau;
    diffang = abs(wrapToPi(x - y - pi));
    ind = ind | diffang <= tau;
    z = epsilon/(2*pi-4*tau)*ones(size(x));
    z(ind) = (1-epsilon)/(4*tau);

end

function R = computeR(a, b, c)

    R = [cos(a)*cos(b) -sin(a)*cos(c) + cos(a)*sin(b)*sin(c) sin(a)*sin(c)+cos(a)*sin(b)*cos(c);
         sin(a)*cos(b) cos(a)*cos(c) + sin(a)*sin(b)*sin(c) -cos(a)*sin(c)+sin(a)*sin(b)*cos(c);
         -sin(b)                -cos(b)*sin(c)                  cos(b)*cos(c)];

end

function y = wrapToPi(x)
    % Makes x between [-pi, pi]
    y = mod(x, sign(x)*2*pi);
    y(abs(y)>pi) = y(abs(y)>pi) - sign(y(abs(y)>pi))*2*pi;
end
