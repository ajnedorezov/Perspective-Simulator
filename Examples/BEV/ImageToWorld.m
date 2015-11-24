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