function [xn, yn] = findnearest (x, y, x0, y0, N)

    % Find the closest point
    d = (x - x0).^2 + (y - y0).^2;
    [~, ind] = min (d);
    
    xn1 = x(ind);
    yn1 = y(ind);

    xn2 = []; 
    yn2 = [];

    if N > 1
        % Continue looking for more closest points excluding the ones
        % already found
        x = [x(1:ind-1); x(ind+1:end)];
        y = [y(1:ind-1); y(ind+1:end)];
        [xn2, yn2] = findnearest (x, y, x0, y0, N-1);
    end

    xn = [xn1; xn2];
    yn = [yn1; yn2];