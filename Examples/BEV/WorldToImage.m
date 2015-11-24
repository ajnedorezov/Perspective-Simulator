function [U,V] = WorldToImage(y,x,params)
    m = params.m-1;
    n = params.n-1;
    alpha = params.alpha;
    dx = params.CameraLocationInWorld(1);
    dy = params.CameraLocationInWorld(2);
    dz = params.CameraLocationInWorld(3);
    theta = params.Theta;
    gamma = params.Gamma;


%     U = ( m/(2*alpha)) * ( atan2(dz*sin(atan2(x-dx, y-dy)), x-dx) - theta + alpha);
%     V = ( n/(2*alpha)) * ( atan2(x-dx, y-dx) - gamma + alpha);

    U = (atan2(y-dy, x-dx) - gamma + alpha) / (2*alpha / n);
    V = (atan2(dz , sqrt((x-dx).^2 + (y-dy).^2)) - theta + alpha) / (2*alpha / m);

end