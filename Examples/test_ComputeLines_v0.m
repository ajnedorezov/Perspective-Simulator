function test_ComputeLines
    clc
    clear all
    close all

    % Read in an image
    im = imread('Test Images\pic002.jpg');
%     im = imread('Test Images\pic003.png');
    movegui(figure, 'northwest')
    imshow(im)   
    
    % Perform edge detection
    greyim = double(rgb2gray(im));
    imedge = edge(greyim, 'sobel');
%     imedge = edge(greyim, 'canny');
    movegui(figure, 'southwest')
    imshow(imedge);
    
    % Find the x & y derivatives
    xdir = imfilter(greyim, [-1 0 1]);
    ydir = imfilter(greyim, [-1 0 1]');
    immag = hypot(xdir, ydir);
    imang = atan2(ydir, xdir);
    
    % Draw the direction of the x & y derivatives for each of the edges.
    [xx, yy] = meshgrid(1:size(greyim,2), 1:size(greyim,1));
    figure(2)
    hold on
    quiver(xx(imedge), yy(imedge), xdir(imedge), ydir(imedge), 'color', 'c')
    
    % Break the image into a much coarser grid and average the x & y
    % derivates within that grid space;
    gridsize = 6;
    [sx, sy] = size(greyim);
    xstartind = round(linspace(1, sy, gridsize));
    ystartind = round(linspace(1, sx, gridsize));
    
    % Compute the average gradient for each grid space
    sector = struct;
    for xind = 1:length(xstartind)-1
        for yind = 1:length(ystartind)-1
            magGradient = immag(ystartind(xind):ystartind(xind+1),...
                                xstartind(yind):xstartind(yind+1));
            angGradient = imang(ystartind(xind):ystartind(xind+1),...
                                xstartind(yind):xstartind(yind+1));
            edges = imedge(ystartind(xind):ystartind(xind+1),...
                           xstartind(yind):xstartind(yind+1));
            sector(xind, yind).MAP = findManhattanAngles(magGradient, angGradient, edges);

            sector(xind, yind).xpos = (xstartind(xind)+xstartind(xind+1))/2;
            sector(xind, yind).ypos = (ystartind(yind)+ystartind(yind+1))/2;
            
            t = sector(xind, yind);
            a = t.MAP.angles(1);
            b = t.MAP.angles(2);
            c = t.MAP.angles(3);
            R = [cos(a)*cos(b) -sin(a)*cos(c) + cos(a)*sin(b)*sin(c) sin(a)*sin(c)+cos(a)*sin(b)*cos(c);
                 sin(a)*cos(b) cos(a)*cos(c) + sin(a)*sin(b)*sin(c) -cos(a)*sin(c)+sin(a)*sin(b)*cos(c);
                 -sin(b)                -cos(b)*sin(c)                  cos(b)*cos(c)];
            quiver(t.xpos, t.ypos, 10*R(1,1), 10*R(1,2), 'color', 'r');
            quiver(t.xpos, t.ypos, 10*R(2,1), 10*R(2,2), 'color', 'g');
            quiver(t.xpos, t.ypos, 10*R(3,1), 10*R(3,2), 'color', 'b');
            fprintf('Angles: %s\n', mat2str(t.MAP.angles*180/pi, 3))
        end
    end
    
    
    % Create the A and b matrix
    A = zeros(numel(sector), 2);
    b = zeros(numel(sector), 1);
    count = 1;
    for xind = 1:length(xstartind)-1
        for yind = 1: length(ystartind)-1
            t = sector(xind, yind);
%             A(count, :) = [-t.ymean t.xmean];
%             b(count) = t.xmean*A(count, :)*[t.xpos t.ypos]';
            A(count, :) = [-cos(t.MAP.angles(1))/cos(t.MAP.angles(2)) 1];
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
        plot(pts(1,:), pts(2,:), 'yx')
        plot(mean(pts(1,:)), mean(pts(2,:)), 'mo')
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
        pts = [pts y];
    end
end

function [MAP] = findManhattanAngles(magGradient, angGradient, edges)
    MAP = struct('likelihood', 0, 'P_edges', [], 'angles', [0 0 0]);
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
    angGradient = angGradient(edges);
    P_ang = 1/(2*pi)*ones([size(P_edge) 5]);
    if isempty(P_edge)
        return
    end

    % Do a coarse search
    alpha = (linspace(-40,40,10))*pi/180;
    beta = (linspace(-40,40,10) - 0)*pi/180;
    gamma = 0;%(linspace(-10,10,3) - 90)*pi/180;
    
    MAP = AngleSearch(alpha, beta, gamma, P_edge, P_ang, angGradient, MAP);
        
    % Do a finer search
    tpi = 10/pi;
    alpha = linspace(MAP.angles(1)-tpi, MAP.angles(1)+tpi, 10);
    beta = linspace(MAP.angles(2)-tpi, MAP.angles(2)+tpi, 10);
    gamma = 0;%linspace(MAP.angles(3)-tpi/4, MAP.angles(3)+tpi/4, 3);
    
    MAP = AngleSearch(alpha, beta, gamma, P_edge, P_ang, angGradient, MAP);
    
end

function MAP = AngleSearch(alpha, beta, gamma, P_edge, P_ang, angGradient, MAP)
    m_u = [0.02 0.02 0.02 0.04 0.9];
    m_u = reshape(m_u, [1 1 numel(m_u)]);
    c = 0.1;
    tau = 6*pi/180;
    
    for a = alpha
        P_ang(:,:,1:3) = c/(2*pi-4*tau);
        angdiff_alpha = abs(wrapToPi(angGradient - a));
        indalpha = angdiff_alpha <= tau;
        angdiff_alpha = abs(wrapToPi(angGradient - a - pi));
        indalpha = indalpha | angdiff_alpha <= tau;
        temp = P_ang(:,:,1);
        temp(indalpha) = (1-c)/(4*tau);
        P_ang(:,:,1) = temp;
        
        for b = beta
            angdiff_beta = abs(wrapToPi(angGradient - b));
            indbeta = angdiff_beta <= tau;
            angdiff_beta = abs(wrapToPi(angGradient - b - pi));
            indbeta = indbeta | angdiff_beta <= tau;
            temp = P_ang(:,:,2);
            temp(indbeta) = (1-c)/(4*tau);
            P_ang(:,:,2) = temp;
            
            for c = gamma
                angdiff_gamma = abs(wrapToPi(angGradient - c));
                indgamma = angdiff_gamma <= tau;
                angdiff_gamma = abs(wrapToPi(angGradient - c - pi));
                indgamma = indgamma | angdiff_gamma <= tau;
                temp = P_ang(:,:,3);
                temp(indgamma) = (1-c)/(4*tau);
                P_ang(:,:,3) = temp;
                
                % Compute the liklihood
                P_onoff = bsxfun(@times, P_edge, P_ang);
                P_e_mu_etc = bsxfun(@times, P_onoff, m_u);

                likelihood = log10(sum(P_e_mu_etc, 3));
                likelihood = sum(likelihood(:));
                
                if likelihood < MAP.likelihood
                    MAP.likelihood = likelihood;
                    MAP.P_edges = P_e_mu_etc;
                    MAP.angles = [a b c];
                end
            end
        end
    end
end

function y = wrapToPi(x)
    % Makes x between 0 & 2*pi
    y = mod(x, sign(x)*2*pi);
    y(abs(y)>pi) = y(abs(y)>pi) - sign(y(abs(y)>pi))*2*pi;
end
