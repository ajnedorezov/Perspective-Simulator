function test_findIntersections
    clc
    clear all
    close all
    
    % Define the equations:
    A = [1 -1; 1 -1; 1  1; 1  1];
    b = [-1 1 -2 2]';
%     A = [2 1;1 3;1 1;3 2];
%     b = [6 8 5 9]';
%     A = [1 -2; 1 -1; 1 2; 1  1];
%     b = [-1 1 -2 2]';
%     A = [-1.07299270072993 0.116788321167883;-3.76923076923077 -2.69230769230769;0.833333333333333 -5.29166666666667;-4.2962962962963 -0.320987654320988;-1.790273556231 -0.0699088145896656;-1.50675675675676 0.506756756756757;-2.51648351648352 0.0659340659340659;-0 0;-2.43786982248521 -0.325443786982249;0.290909090909091 -1.03333333333333;-0.613636363636364 1.62878787878788;-0.62406015037594 0.225563909774436;-0.401234567901235 0.549382716049383;-0.96 -0.365;-1.61832061068702 0.366412213740458;-0.782608695652174 1.35869565217391;-3.0506329113924 0.392405063291139;0.0833333333333333 -0.0961538461538462;-0.631868131868132 0.840659340659341;-0.845528455284553 -0.227642276422764;-0.0983606557377049 0.254098360655738;0.553072625698324 0.631284916201117;-0.0977011494252874 0.396551724137931;-0.818181818181818 0.391608391608392;1.46896551724138 0.441379310344828];
%     b = [-3.57099472534498;800.236686390533;2957.49045138889;58.6178936137784;4.84750695207916;-63.5774287801315;-14.807631928511;0;88.5091558418823;184.064848484849;-89.4044134527089;-17.7763016563966;0.176345069349189;73.25185;-62.6592855894179;-180.839437618147;-241.070821983656;-0.661057692307692;-2.71597633136093;50.6809438826096;-5.26316850309057;120.970412908461;6.91572532699168;-62.8928553963519;213.633674197384];
    
    % Plot each of the constraint lines
    movegui(figure, 'northwest')
    hold all
    axis equal
    x = linspace(-500,500);
    for l = 1:size(A,1)
        liney = (-A(l,1)*x + b(l)) / A(l,2);
        plot(x, liney)
    end
    
    % Compute all the line intersections
    tic
    intersectPts = findVert(A, b);
    toc
    % Plot the intersection points
    plot(intersectPts(1,:), intersectPts(2,:), 'o')

end

function pts = findVert(A, b)
    pts = [];
    
    [numConstraints, numVariables] = size(A);
    
    % If num constraints > num variables
    if numConstraints > numVariables
        A = [A eye(numConstraints)];
        [numConstraints, numVariables] = size(A);
%         if size(A,1) ~= size(A,2)
%             error('Unable to make A square')
%         end
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

