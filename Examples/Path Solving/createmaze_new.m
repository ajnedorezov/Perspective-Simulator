function createmaze_new()
%maze(row,col,pattern)
% usage  thyme = maze(30,45,'c');
% row - number of rows in the maze
% col - number of column in the maze
% pattern - random(r), vertical(v), horizontal(h), checkerboard(c), spiral(s), burst(b)

% Written by Rodney Meyer
% rodney_meyer@yahoo.com
%
% Construct graph system for maze. The graph entities are an id for each
% intersection(id), the physical row(rr) and column(cc) of the
% intersection, membership to a connected region (state), and a link to 
% adjacent intersections(ptr_up ptr_down ptr_left ptr_right). 
% Prior to "make_pattern" the maze has all of the walls intact and
% there are row*col of unconnected states. After "make_pattern" some of the
% walls are broken down and there is only one connected state for the maze.
% A broken wall(allowed passage) in some direction is signified by a negative
% value of the pointer in that direction. A solid wall(unallowed passage) 
% in some direction is signified by a positive value of the pointer in that 
% direction. The absolute value of the pointer is the id of the
% intersection in that direction.

% rand('state',sum(100*clock))
row = 5;
col = 5;
pattern = 'c';

% rand('state',1)
rng('state',4)

[cc,rr]=meshgrid(1:col,1:row);
state = reshape([1:row*col],row,col); % state identifies connected regions
id = reshape([1:row*col],row,col); % id identifies intersections of maze

% create pointers to adjacent intersections
ptr_left = zeros(size(id));
ptr_up = zeros(size(id));
ptr_right = zeros(size(id));
ptr_down = zeros(size(id));

ptr_left(:,2:size(id,2)) = id(:,1:size(id,2)-1);
ptr_up(2:size(id,1),:) = id(1:size(id,1)-1,:);
ptr_right(:,1:size(id,2)-1) = id(:,2:size(id,2));
ptr_down(1:size(id,1)-1,:) = id(2:size(id,1),:);

% sort graph entities by id
the_maze = cat(2,reshape(id,row*col,1),reshape(rr,row*col,1),reshape(cc,row*col,1),reshape(state,row*col,1),...
    reshape(ptr_left,row*col,1),reshape(ptr_up,row*col,1),reshape(ptr_right,row*col,1),reshape(ptr_down,row*col,1)  );

the_maze = sortrows(the_maze);

id=the_maze(:,1);
rr=the_maze(:,2);
cc=the_maze(:,3);
state=the_maze(:,4);
ptr_left=the_maze(:,5);
ptr_up=the_maze(:,6);
ptr_right=the_maze(:,7);
ptr_down=the_maze(:,8);
clear the_maze;

% create a random maze
[state, ptr_left, ptr_up, ptr_right, ptr_down]=...
    make_pattern(row,col,pattern,id, rr, cc, state, ptr_left, ptr_up, ptr_right, ptr_down);

% show maze
h=figure('KeyPressFcn',@move_spot,'color','white');
show_maze(row, col, rr, cc, ptr_left, ptr_up, ptr_right, ptr_down,h);


f = getframe(gcf);
delete(gcf);
Im = im2double(rgb2gray(f.cdata));
imsize = size(Im);


    % Create the edge map
    sigma = 1;
%     smIm = abs(imfilter(Im, fspecial('laplacian'), 'same', 'replicate'));
%     smIm = imfilter(smIm, fspecial('gaussian', ceil(sigma*[3 3]), sigma), 'same', 'replicate');
    smIm = double(Im > 0.5);
%     smIm = smIm/2;
%     smIm(50:70, 65:85) = 1;
%     smIm(340:360, 500:520) = 1;

    

    se = strel('ball',50,50);
    smIm = 50-imdilate(1-smIm,se);
    
    LeftRight = [zeros(size(smIm,1),1) diff(smIm, [], 2)];
    UpDown = [zeros(1,size(smIm,2)); diff(smIm, [], 1)];

    [xx,yy] = meshgrid(1:imsize(2), 1:imsize(1));
%     dx = xx - 510-1;
%     dy = yy - 350-1;
    dx = 510 - xx;
    dy = 350 - yy;
    newMag =  sqrt(dx.*dx + dy.*dy);
%     newMag = 1 - newMag / max(newMag(:));
    newMag = newMag / max(newMag(:));
% 
%     smIm = 0.25*smIm + 0.75*newMag;
    
    % Display the figure
    hf = figure;
    ha(1) = subplot(221);
    imshow(Im)
    title('Original Image')
    ha(2) = subplot(222);
    hIm = imshow(smIm,[]);
    hold on
    title('Energy function with current spline')    
    
    % Get the initial snake coordinates
    x_s = nan; y_s = nan;
    hLine = plot(ha(2), x_s, y_s, 'ro-');
%     theta = linspace(0,2*pi,10);
%     x_s = [70 linspace(75,510,10) 515 515 70 70];
%     y_s = [60 linspace(60,350,10) 350 370 370 60];
    x_s = linspace(75,510,10);
    y_s = linspace(60,350,10);
    set(hLine, 'XData', x_s, 'YData', y_s);
    
    % Upsample & create a spline
    steps = 0:(length(x_s)-1);
    newSteps = 0:0.05:(length(x_s)-1);
    pp = spline(steps,[x_s' y_s']', newSteps);
    x_s = pp(1,:)';
    y_s = pp(2,:)';
    hSpline = plot(x_s,y_s,'b');
    
    % Create the GVF
    mu = 0.2;%1;
    smIm = padarray(smIm, [1 1], 'symmetric', 'both');
    [fx,fy] = gradient(smIm);

    u = fx;
    v = fy;
    dx = 510-75;
    dy = 350-60;
    xind = 75:510;
    yind = 60:350;
%     u(yind,xind) = fx(yind,xind) + 0.25*dx/hypot(dx,dy);
%     v(yind,xind) = fy(yind,xind) + 0.25*dy/hypot(dx,dy);
    newMag = padarray(newMag, [1 1], 'symmetric', 'both');
    [fx2,fy2] = gradient(newMag);
%     u = fx + fx2;%0.25*fx2./hypot(fx2,fy2);
%     v = fy + fy2;%0.25*fy2./hypot(fx2,fy2);
%     edgeX = 0.5*fx2./(hypot(fx2,fy2) + eps);
%     edgeY = 0.5*fy2./(hypot(fx2,fy2) + eps);
%     u(Im > 0.5) = edgeX(Im > 0.5);
%     v(Im > 0.5) = edgeY(Im > 0.5);
%     gradMag = fx.*fx + fy.*fy;
    gradMag = u.*u + v.*v;
        
%     gradMag(50:70, 65:85) = 0;
%     gradMag(340:360, 500:520) = 0;
    
    for n = 1:80
        u = padarray(u(2:end-1, 2:end-1), [1 1], 'symmetric', 'both');
        v = padarray(v(2:end-1, 2:end-1), [1 1], 'symmetric', 'both');
        u = u + mu*4*del2(u) - gradMag.*(u-fx);
        v = v + mu*4*del2(v) - gradMag.*(v-fy);
    end
    u = u(2:end-1,2:end-1);
    v = v(2:end-1,2:end-1);
    
    maxEdge = max(max(hypot(u,v))); %0.25
    edgeX = maxEdge*fx2./(hypot(fx2,fy2) + eps);
    edgeY = maxEdge*fy2./(hypot(fx2,fy2) + eps);
    
% %     u(Im > 0.5) = u(Im > 0.5)+edgeX(Im > 0.5);
% %     v(Im > 0.5) = v(Im > 0.5)+edgeY(Im > 0.5);

%     u = zeros(size(u)); v = zeros(size(v));
    clockwise = true;
%     clockwise = false;
    if clockwise 
        v(LeftRight < 0) = -maxEdge/2;    u(LeftRight < 0) = 0;
        v(LeftRight > 0) = maxEdge/2;     u(LeftRight < 0) = 0;
        v(UpDown < 0) = 0;              u(UpDown < 0) = maxEdge/2;
        v(UpDown > 0) = 0;              u(UpDown > 0) = -maxEdge/2;
    else
        v(LeftRight < 0) = maxEdge/2;    u(LeftRight < 0) = 0;
        v(LeftRight > 0) = -maxEdge/2;     u(LeftRight < 0) = 0;
        v(UpDown < 0) = 0;              u(UpDown < 0) = -maxEdge/2;
        v(UpDown > 0) = 0;              u(UpDown > 0) = maxEdge/2;
    end    

% 
%     v(LeftRight < 0) = -maxEdge;    u(LeftRight < 0) = 0;
%     v(LeftRight > 0) = maxEdge;     u(LeftRight < 0) = 0;
%     v(UpDown < 0) = 0;              u(UpDown < 0) = maxEdge;
%     v(UpDown > 0) = 0;              u(UpDown > 0) = -maxEdge;

    u(Im < 0.5) = edgeX(Im < 0.5);
    v(Im < 0.5) = edgeY(Im < 0.5);
    
    magGVF = hypot(u,v) + 1e-10;
    fx = u./magGVF;
    fy = v./magGVF;
    
%     fx2(Im>0.5) = 0;
%     fy2(Im>0.5) = 0;
    
% %     figure, quiver(u,v), axis ij, axis image
%     ha(3) = subplot(223); hq = quiver(fx,fy); axis ij, axis image
% %     ha(3) = subplot(223); hq = quiver(fx2,fy2); axis ij, axis image
%     title('Gradients')

    ha(3) = subplot(223); imagesc(magGVF);

    % Create the components of the Euler equation
    % [Tension, rigidity, stepsize, energy portion]
    alpha = 0.1;% 0.4;%0.5; 
    beta = 0.0;%0.5;
    gamma = 1;
    kappa = 0.96;
    A = imfilter(eye(length(newSteps)), [beta -alpha-4*beta 2*alpha+6*beta -alpha-4*beta beta], 'same', 'conv', 'circular');

    % Compute the inverse of A by LU decompositions since it is a
    % pentadiagonal banded matrix
    [L,U] = lu(A + gamma*eye(size(A)));
    invA = inv(U) * inv(L);
        
    ha(4) = subplot(224);
    imagesc(smIm), hold on, title('Results')
    % Iteratively solve the Euler equations for x & y
    set([hLine], 'Visible', 'off')
    try
        set([hq], 'Visible', 'off')
    end
    corder = jet(8);
    count = 1;
    for n = 1:2000 %400
        newx = gamma*x_s + kappa*interp2(fx, x_s, y_s, '*linear', 0);
        newy = gamma*y_s + kappa*interp2(fy, x_s, y_s, '*linear', 0);
        
%         newx([1 end]) = [75 510];
%         newy([1 end]) = [60 350];
        
        x_s = invA*newx;
        y_s = invA*newy;
        
        % Redistribute the points along the curve
        x_s([1 end]) = [75 510];
        y_s([1 end]) = [60 350];   
        dStep = cumsum(hypot([0; diff(x_s)],[0; diff(y_s)]));
        newStep = linspace(rand/max(dStep),max(dStep),length(dStep))';
%         dStep = cumsum(hypot(diff(x_s),diff(y_s)));
%         newStep = linspace(rand/max(dStep),max(dStep),length(dStep))';
        x_s = interp1(dStep,x_s,newStep);
        y_s = interp1(dStep,y_s,newStep);
        
        set(hSpline, 'XData', x_s, 'YData', y_s, 'Marker', 'o');
        drawnow
%         if any(n == [1 10 20 50 100:100:400])
%             subplot(224), hold on
% %             plot(x_s([1:end 1]), y_s([1:end 1]), 'Color', corder(count,:))
%             plot(x_s, y_s, 'Color', corder(count,:), 'Marker', '.')
%             count = count + 1;
%         end            
%         pause(0.1)
    end
%     set(hSpline, 'Color', [0 0.5 0])
    
    
%     ha(4) = subplot(224);
%     imshow(Im), hold on,
%     plot(x_s([1:end 1]),y_s([1:end 1]),'r-')
    try
        set(hq, 'Visible', 'on')
    end
    
    linkaxes(ha, 'xy')
    
%     figure, quiver(u./hypot(u,v),v./hypot(u,v)), axis ij, axis image
    figure, quiver(u,v), axis ij, axis image
    
%     newIm = double(Im > 0.5);
%     dx = [zeros(size(newIm,1),1) diff(newIm, [], 2)];
%     dy = [zeros(1,size(newIm,2)); diff(newIm, [], 1)];
%     
%     newIm(dx<0) = 2; % left - down
%     newIm(dx>0) = 3; % right - up
%     newIm(dy<0) = 4; % up - left
%     newIm(dy>0) = 5; % down - right
    
%     figure, imagesc(newIm);
    
return

function move_spot(src,evnt)
assignin('caller','key',evnt.Key)
return


function show_maze(row, col, rr, cc, ptr_left, ptr_up, ptr_right, ptr_down,h)
figure(h)
line([.5,col+.5],[.5,.5]) % draw top border
line([.5,col+.5],[row+.5,row+.5]) % draw bottom border
line([.5,.5],[1.5,row+.5]) % draw left border
line([col+.5,col+.5],[.5,row-.5])  % draw right border
for ii=1:length(ptr_right)
    if ptr_right(ii)>0 % right passage blocked
        line([cc(ii)+.5,cc(ii)+.5],[rr(ii)-.5,rr(ii)+.5]);
        hold on
    end
    if ptr_down(ii)>0 % down passage blocked
        line([cc(ii)-.5,cc(ii)+.5],[rr(ii)+.5,rr(ii)+.5]);
        hold on
    end
    
end
axis equal
axis([.5,col+.5,.5,row+.5])
axis off
set(gca,'YDir','reverse')

h = get(gca, 'Children');
set(h, 'LineWidth', 5)
return




function [state, ptr_left, ptr_up, ptr_right, ptr_down]=make_pattern(row,col,pattern,id, rr, cc, state, ptr_left, ptr_up, ptr_right, ptr_down)

while max(state)>1 % remove walls until there is one simply connected region
    tid=ceil(col*row*rand(15,1)); % get a set of temporary ID's
    cityblock=cc(tid)+rr(tid); % get distance from the start
    is_linked=(state(tid)==1); % The start state is in region 1 - see if they are linked to the start
    temp = sortrows(cat(2,tid,cityblock,is_linked),[3,2]); % sort id's by start-link and distance
    tid = temp(1,1); % get the id of the closest unlinked intersection
    
    % The pattern is created by selective random removal of vertical or 
    % horizontal walls as a function of position in the maze. I find the
    % checkerboard option the most challenging. Other patterns can be added
    switch upper(pattern) 
    case 'C' % checkerboard
        dir = ceil(8*rand);
        nb=3;
        block_size =  min([row,col])/nb;
        while block_size>12
            nb=nb+2;
            block_size =  min([row,col])/nb;
        end
        odd_even = (ceil(rr(tid)/block_size)*ceil(col/block_size) + ceil(cc(tid)/block_size));
        if odd_even/2 == floor(odd_even/2)
            if dir>6
                dir=4;
            end
            if dir>4
                dir=3;
            end
        else
            if dir>6
                dir=2;
            end
            if dir>4
                dir=1;
            end
        end
    case 'B' % burst
        dir = ceil(8*rand);
        if abs((rr(tid)-row/2))<abs((cc(tid)-col/2))
            if dir>6
                dir=4;
            end
            if dir>4
                dir=3;
            end
        else
            if dir>6
                dir=2;
            end
            if dir>4
                dir=1;
            end
        end
    case 'S' %spiral
        dir = ceil(8*rand);
        if abs((rr(tid)-row/2))>abs((cc(tid)-col/2))
            if dir>6
                dir=4;
            end
            if dir>4
                dir=3;
            end
        else
            if dir>6
                dir=2;
            end
            if dir>4
                dir=1;
            end
        end
    case 'V'
        dir = ceil(8*rand);
        if dir>6
            dir=4;
        end
        if dir>4
            dir=3;
        end
    case 'H'
        dir = ceil(8*rand);
        if dir>6
            dir=2;
        end
        if dir>4
            dir=1;
        end
        otherwise % random
        dir = ceil(4*rand);
    end
    
    % after a candidate for wall removal is found, the candidate must pass
    % two conditions. 1) it is not an external wall  2) the regions on
    % each side of the wall were previously unconnected. If successful the
    % wall is removed, the connected states are updated to the lowest of
    % the two states, the pointers between the connected intersections are
    % now negative.
    switch dir
    case -1
        
    case 1
        if ptr_left(tid)>0 & state(tid)~=state(ptr_left(tid))
            state( state==state(tid) | state==state(ptr_left(tid)) )=min([state(tid),state(ptr_left(tid))]);
            ptr_right(ptr_left(tid))=-ptr_right(ptr_left(tid));
            ptr_left(tid)=-ptr_left(tid);
        end
    case 2
        if ptr_right(tid)>0 & state(tid)~=state(ptr_right(tid))
            state( state==state(tid) | state==state(ptr_right(tid)) )=min([state(tid),state(ptr_right(tid))]);
            ptr_left(ptr_right(tid))=-ptr_left(ptr_right(tid));
            ptr_right(tid)=-ptr_right(tid);
        end
    case 3
        if ptr_up(tid)>0 & state(tid)~=state(ptr_up(tid))
            state( state==state(tid) | state==state(ptr_up(tid)) )=min([state(tid),state(ptr_up(tid))]);
            ptr_down(ptr_up(tid))=-ptr_down(ptr_up(tid));
            ptr_up(tid)=-ptr_up(tid);
        end
    case 4
        if ptr_down(tid)>0 & state(tid)~=state(ptr_down(tid))
            state( state==state(tid) | state==state(ptr_down(tid)) )=min([state(tid),state(ptr_down(tid))]);
            ptr_up(ptr_down(tid))=-ptr_up(ptr_down(tid));
            ptr_down(tid)=-ptr_down(tid);
        end
    otherwise
        dir
        error('quit')
    end
    
end
return