function [sys, x0, str, ts, simStateCompliance] = simulation_display(t, ~, u, flag, ax, worldfile)
    %simulation_display S-function that displays a figure with virtual reality 
    %   canvas combined with three graphs.
    %   This MATLAB function is designed to be used in a Simulink S-function block.
    %   The S-function block inputs are displayed under the virtual canvas in 
    %   three graphs.
    %   Block parameters define graph ranges and the associated virtual scene file. 
    %   It is expected that the same scene is driven by a VR Sink block
    %   present in the same model.

    %   Copyright 1998-2010 HUMUSOFT s.r.o. and The MathWorks, Inc.
    %   $Revision: 1.1.6.2 $ $Date: 2010/02/17 19:07:04 $ $Author: batserve $

    % Store the block handle
    blockHandle = gcbh;

    switch flag

      % Initialization
      case 0
        [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes(ax);
        SetBlockCallbacks(blockHandle, worldfile);

      % Update
      case 2
        sys = mdlUpdate(t, u, ax, blockHandle);

      % Start
      case 'Start'
        LocalBlockStartFcn(blockHandle, worldfile)

      % Stop
      case 'Stop'
        LocalBlockStopFcn(blockHandle)

      % NameChange
      case 'NameChange'
        LocalBlockNameChangeFcn(blockHandle)

      % CopyBlock, LoadBlock
      case { 'CopyBlock', 'LoadBlock' }
        LocalBlockLoadCopyFcn(blockHandle)

      % DeleteBlock
      case 'DeleteBlock'
        LocalBlockDeleteFcn(blockHandle)

      % DeleteFigure
      case 'DeleteFigure'
        LocalFigureDeleteFcn();

      % Unused flags
      case { 3, 9 }
        sys = [];

      % Unexpected flags
      otherwise
         if ischar(flag)
           DAStudio.error('VR:unhandledflag', flag);
         else
           DAStudio.error('VR:unhandledflag', num2str(flag));
         end

    end
end
% end switchyard



%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================

function [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes(ax)

    if length(ax)~=6
      DAStudio.error('VR:axislimitsmustbedefined');
    end

    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 0;
    sizes.NumInputs      = 3;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);

    x0  = [];
    str = [];
    ts  = [-1 0];

    % specify that the simState for this s-function is same as the default
    simStateCompliance = 'DefaultSimState';
end
% end mdlInitializeSizes



%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================

function sys = mdlUpdate(t, u, ax, block, varargin)

    % always return empty, there are no states
    sys = [];

    % Locate the figure window associated with this block.  If it's not a valid
    % handle (it may have been closed by the user), then return.
    FigHandle = Get_3GFigure(block);
    if ~ishandle(FigHandle)
       return;
    end

    % Get UserData of the figure
    ud = get(FigHandle, 'UserData');
    
    % Capture an image of the current scene
    IC = ud.ImageCanvas;
    im = capture(IC(1));

    % Performe the BEV transform
    imBEV = zeros(length(ud.BEV.yg), length(ud.BEV.xg), size(im,3));
    [xx,yy] = meshgrid(ud.BEV.xg, ud.BEV.yg);
    for d = 1:3
%         imBEV(:,:,d) = interp2(u, v, double(im(:,:,d)), X, Y);
%         tempIm = griddata(ud.BEV.X,ud.BEV.Y, double(im(ud.BEV.rHorizon:end, :, d))/255, ud.BEV.xg, ud.BEV.yg, 'linear');
        interpIm = double(im(ud.BEV.rHorizon:end, :, d))/255;
        

        F = TriScatteredInterp(ud.BEV.X(:), ud.BEV.Y(:), interpIm(:));
        imBEV(:,:,d) = rot90(F(xx,yy),2);
    end
    
    imBEV = uint8(imBEV*255);

    if ud.framecount  == 0
        % Initialize the previous panorama image
        ud.PreviousIm = ones(size(imBEV));
    end
    
    % Store the current panoramic image as the previous image
    ud.PreviousIm = imBEV;
    ud.Frames{max(1,ud.framecount)} = imBEV;

    % Draw the BEV image on the display
    imshow(imBEV, 'Parent', ud.G1_Axes)
    grid on
    
    % flush event queue
    drawnow;

    if ud.framecount > 0 
        % Store the current display in the video
        currFrame = getframe(ud.BEViewer);
        writeVideo(ud.Video, currFrame);
    end
    
    ud.framecount = ud.framecount + 1;

    set(FigHandle, 'UserData', ud);
end
% end mdlUpdate



%=============================================================================
% LocalBlockStartFcn
% Function that is called when the simulation starts.
% Initialize the figure.
%=============================================================================

function LocalBlockStartFcn(block, worldfile)

    % get the figure associated with this block, create a figure if it doesn't
    % exist
    FigHandle = Get_3GFigure(block);
    if ~ishandle(FigHandle)
      FigHandle = Create_3GFigure(block, worldfile);
    end

    % get UserData of the figure
    ud = get(FigHandle, 'UserData');

    ud.XData  = [];
    ud.Y1Data = [];
    ud.Y2Data = [];
    ud.Y3Data = [];

    % Create the video file the panorama images will be saved to
    ud.Video = VideoWriter('BEV-Video.avi');
    set(ud.Video, 'FrameRate', 8)
    open(ud.Video);
    ud.framecount = 0;
    ud.ImHistory = [];

    set(FigHandle, 'UserData', ud);
end
% end LocalBlockStartFcn



%=============================================================================
% LocalBlockStopFcn
% At the end of the simulation, set the graph's data to contain
% the complete set of points that were acquired during the simulation.
% Recall that during the simulation, the lines are only small segments from
% the last time step to the current one.
%=============================================================================

function LocalBlockStopFcn(block)

    FigHandle = Get_3GFigure(block);
    if ishandle(FigHandle)

      % Get UserData of the figure.
      ud = get(FigHandle, 'UserData');

      close(ud.Video);

    end
end
% end LocalBlockStopFcn



%=============================================================================
% LocalBlockNameChangeFcn
% Function that handles name changes of the block.
%=============================================================================

function LocalBlockNameChangeFcn(block)

    % get the figure associated with this block, if it's valid, change
    % the name of the figure
    FigHandle = Get_3GFigure(block);
    if ishandle(FigHandle)
      set(FigHandle, 'Name', BlockFigureTitle(block));
    end
end
% end LocalBlockNameChangeFcn



%=============================================================================
% LocalBlockLoadCopyFcn
% This is the block LoadFcn and CopyFcn. Initialize the block's
% UserData such that a figure is not associated with the block.
%=============================================================================

function LocalBlockLoadCopyFcn(block)

    Set_3GFigure(block, -1);

end
% end LocalBlockLoadCopyFcn



%=============================================================================
% LocalBlockDeleteFcn
% This is the block DeleteFcn. Delete the block's figure window,
% if present, upon deletion of the block.
%=============================================================================

function LocalBlockDeleteFcn(block)

    % Get the figure handle associated with the block, if it exists, delete
    % the figure.
    FigHandle = Get_3GFigure(block);
    if ishandle(FigHandle)
      delete(FigHandle);
      Set_3GFigure(block, -1);
    end
end
% end LocalBlockDeleteFcn



%=============================================================================
% LocalFigureDeleteFcn
% This is the figure DeleteFcn. The figure window is
% being deleted, update the block UserData to reflect the change.
%=============================================================================

function LocalFigureDeleteFcn()

    % Get the block associated with this figure and set its figure to -1
    ud = get(gcbf, 'UserData');
    if ~isempty(ud) 
      Set_3GFigure(ud.Block, -1);
    end
end
% end LocalFigureDeleteFcn



%=============================================================================
% Get_3GFigure
% Retrieves the figure window associated with this S-function block
% from the block's parent subsystem's UserData.
%=============================================================================

function FigHandle = Get_3GFigure(block)

    if strcmp(get_param(block, 'BlockType'), 'S-Function')
      block = get_param(block, 'Parent');
    end

    FigHandle = get_param(block, 'UserData');
    if isempty(FigHandle)
      FigHandle = -1;
    end
end
% end Get_3GFigure



%=============================================================================
% Set_3GFigure
% Stores the figure window associated with this S-function block
% in the block's parent subsystem's UserData.
%=============================================================================

function Set_3GFigure(block, FigHandle)

    if strcmp(get_param(bdroot, 'BlockDiagramType'), 'model')
      if strcmp(get_param(block, 'BlockType'), 'S-Function')
        block = get_param(block, 'Parent');
      end

      set_param(block, 'UserData', FigHandle);
    end
end
% end Set_3GFigure



%=============================================================================
% Create_3GFigure
% Creates the figure window associated with this S-function block.
%=============================================================================

function FigHandle = Create_3GFigure(block, worldfile)

    % The figure doesn't exist, create one
    FigHandle = figure('Units',          'pixels', ...
                       'Position',       [0 0 640 480], ...
                       'Color',          [0.314 0.314 0.314], ...
                       'Name',           BlockFigureTitle(block), ...
                       'Tag',            'simulation_display_fig', ...
                       'NumberTitle',    'off', ...
                       'IntegerHandle',  'off', ...
                       'Toolbar',        'figure', ...
                       'Resize',         'off',...
                       'DeleteFcn',      'simulation_display([], [], [], ''DeleteFigure'', [], [])');

    % Store the block handle in the figure UserData
    ud.Block = block;

    % Move the figure to the northwest corner
    movegui(FigHandle, 'northwest');
    ud.DriverView = FigHandle;

    % Create two uipanels to house the images, one hidden which show all 8
    % viewers and 1 visibile that show the panorama image
    ud.panopanel = uipanel('Parent', FigHandle, 'Visible', 'on');

    % Open the vrworld if not open already
    vr_world = vrworld(worldfile);
    if ~isopen(vr_world)
      open(vr_world);
    end
    ud.vr_world = vr_world;

    % Draw a canvas on the figure to see what the current state of the
    % simulation is
    ud.ImageCanvas = vr.canvas(vr_world, 'Parent', FigHandle,...
            'Units', 'pixels', ...
            'CameraBound', 'off',...
            'Position', [0 0 640 480],...get(a, 'Position'),...
            'NavPanel', 'none',...
            'Viewpoint', 'Main_Driver_View');


    % Create an axes that displays birds eye view image
    FigHandle = figure('Resize', 'off', 'Position', [0 0 640 800]);
    movegui(FigHandle, 'northeast');
    ud.BEViewer = FigHandle;

    ud.G1_Axes = axes('Parent', FigHandle);
    figure(FigHandle)
    title(ud.G1_Axes, 'Bird''s Eye View');  


    % Initialize some parameters
    imsize = size(capture(ud.ImageCanvas));
    alpha_tot = 30*pi/180;
    den = sqrt((imsize(2)-1)^2+(imsize(1)-1)^2);
    alpha = atan( (imsize(2)-1)/den * tan(alpha_tot) );
    ud.params = struct('alpha', alpha,...
                       'CameraLocationInWorld', [0 0 10],...
                       'Gamma', 0,...
                       'Theta', 0*pi/180,...atan(0.5/20),...
                       'm', imsize(1),...
                       'n', imsize(2));

    rHorizon = ceil((ud.params.m-1)/(2*ud.params.alpha)*(ud.params.alpha-ud.params.Theta)+1)+10;%-30;
    [v,u] = meshgrid((1:imsize(2))-1, (rHorizon):imsize(1));
    [X,Y] = ImageToWorld(v, u, ud.params);

    step = 0.2;
%     xg = 34:-step:-26; % Need to go top to bottom so the image won't get
%                    % mirrored vertically.
%     yg = (10:step:160)';
    xg = fliplr(linspace(-30, 30, imsize(1)));
    yg = linspace(9, 120, imsize(2))';
    
    ud.BEV.rHorizon = rHorizon;
    ud.BEV.X = X;
    ud.BEV.Y = Y;
    ud.BEV.step = step;
    ud.BEV.xg = xg;
    ud.BEV.yg = yg;

    % Associate the figure with the block, and set the figure's UserData.
    Set_3GFigure(block, FigHandle);
    set(FigHandle, 'UserData', ud, 'HandleVisibility', 'callback');
end
% end Create_3GFigure



%=============================================================================
% BlockFigureTitle
% String for figure window title
%=============================================================================

function title = BlockFigureTitle(block)
      iotype = get_param(block, 'iotype');
      if strcmp(iotype, 'viewer')
        title = viewertitle(block, false);
      else
        title = get_param(block, 'Name');
      end
end
% end BlockFigureTitle



%=============================================================================
% SetBlockCallbacks
% This sets the callbacks of the block if it is not a reference.
%=============================================================================

function SetBlockCallbacks(block, worldfile)

    % the actual source of the block is the parent subsystem
    parent = get_param(block, 'Parent');

    % set the callbacks for the block so that it has the proper functionality
    callbacks = {
    'CopyFcn',       'simulation_display([], [], [], ''CopyBlock'', [], [])'; ...
    'DeleteFcn',     'simulation_display([], [], [], ''DeleteBlock'', [], [])'; ...
    'LoadFcn',       'simulation_display([], [], [], ''LoadBlock'', [], [])'; ...
    'StartFcn',      sprintf('simulation_display([], [], [], ''Start'', [], ''%s'')', worldfile); ...
    'StopFcn'        'simulation_display([], [], [], ''Stop'', [], [])'; ...
    'NameChangeFcn', 'simulation_display([], [], [], ''NameChange'', [], [])'; ...
    };

    for i=1:length(callbacks)
      if ~strcmp(get_param(parent, callbacks{i,1}), callbacks{i,2})
        set_param(parent, callbacks{i,1}, callbacks{i,2})
      end
    end
end
% end SetBlockCallbacks


%=============================================================================
%% Additional callbacks
%=============================================================================
function [X,Y] = ImageToWorld(v,u, params)

    m = params.m-1;
    n = params.n-1;
    alpha = params.alpha;
    dx = params.CameraLocationInWorld(1);
    dy = params.CameraLocationInWorld(2);
    dz = params.CameraLocationInWorld(3);
    theta = params.Theta;
    gamma = params.Gamma;

    X = dz * cot(theta - alpha + u.*(2*alpha/m)) .* sin(gamma - alpha + v.*(2*alpha / n)) + dx;
    Y = dz * cot(theta - alpha + u.*(2*alpha/m)) .* cos(gamma - alpha + v.*(2*alpha / n)) + dy;
end
% end ImageToWorld