%% Set some world parameters
numlanes = 3;
lanewidth = 3; % m

lanemarkerwidth = 0.15/2; %m  (6in)

numCars = 4;
carlength = 4.5;
carwidth = 1.7;
carheight = 1.25;


mi2m = 1609.34;
lookAheadDist = 0.5 * mi2m; % meters

maxDist = 1*mi2m;

%% Set some camara parameters
FOV.distance = 100; % m
FOV.angle = 160; % deg

FOV.camHeight = carheight; %m

%% Make the figure
% Create a figure
hf = clf(figure(1));
% set(hf, 'Position', [100 100 512 512]);

% Create an axis
ha = axes('Parent', hf, 'Color', 'c', 'ydir', 'reverse', 'Units', 'pixels', 'Position', [1 1 512 512]);

axis image
axis off

% Turn off the tick marks/labels
% set(ha, 'xtick', [], 'ytick', [], 'ztick', []);
xlabel('Down Range (m)');
ylabel('Cross Range (m)');
zlabel('Altitude (m)');

%% Draw the ground plane
gpx = [-10 maxDist maxDist -10];
gpy = [-100 -100 100 100];
gpz = [0 0 0 0];

hgp = patch(gpx, gpy, gpz, [0 0.5 0], 'edgecolor', 'none');

%% Draw the background plane
sx = maxDist*[1 1 1 1];
sy = 10000*[-1 1 1 -1];
sz = 10000*[-1 -1 1 1];

hs = patch(sx, sy, sz, [0 0.5 0.75], 'edgecolor', 'none');

%% Draw the driving lanes
dlpx = [0 0 maxDist maxDist];
dlpy = [0 lanewidth lanewidth 0];
dlpz = [0 0 0 0] + 0.01;

dlmx = [0 0 maxDist maxDist];
dlmy = [0 lanemarkerwidth lanemarkerwidth 0];
dlmz = [0 0 0 0] + 2*0.01;

hlm = patch(dlmx, 2*dlmy - lanemarkerwidth, dlmz, [1 0.95 0], 'edgecolor', 'none');
for n = 1:numlanes
    hdl(n) = patch(dlpx, dlpy + (n-1)*lanewidth, dlpz, [0.3 0.3 0.3], 'edgecolor', 'none');
    hlm(n+1) = patch(dlmx, dlmy + n*lanewidth - lanemarkerwidth/2, dlmz, [1 0.95 0], 'edgecolor', 'none');
end

%% Draw some "Cars"
cpx = carlength*[0.5 0.5 0.5 0.5 0.5; 0 0 1 1 0; 0 0 1 1 0; 0.5 0.5 0.5 0.5 0.5];
cpy = carwidth *[0.5 0.5 0.5 0.5 0.5; 0 1 1 0 0; 0 1 1 0 0; 0.5 0.5 0.5 0.5 0.5];
cpz = carheight*[0 0 0 0 0; 0 0 0 0 0; 1 1 1 1 1; 1 1 1 1 1] + 3*0.01;

carlane = randi(numlanes,1,numCars)-1;
% carlane = [2 0 1 1];
for n = 1:numCars
    hc(n) = surface(cpx + 25*n, cpy + (carlane(n)+0.5)*lanewidth - carwidth/2, cpz, 'FaceColor', [0.5 0 0], 'edgecolor', 'none');
end

%% Set the initial camara pos
% campos([0 lanewidth/2 FOV.camHeight]); % Put the camera at the origin
% campos([0 FOV.camHeight 10]); % Put the camera at the origin
posvec = [0 lanewidth/2 FOV.camHeight];
campos(posvec);
targvec = posvec + [lookAheadDist 0 0];
camtarget(targvec);  % The camera looks along the +Z axis
camproj('perspective')
camva(45)
% camva(160)

%% Start driving forward

for n = 1:25*(numCars+1)
    if ~ishandle(ha)
        break
    end
    posvec(1:2) = [n lanewidth*sin(n/25) + 3*lanewidth/2];
    targvec(1:2) = posvec(1:2) + lookAheadDist*[1 lanewidth/25*cos(n/25)];
    campos(posvec);
    camtarget(targvec);
    
    pause(0.2);
end
