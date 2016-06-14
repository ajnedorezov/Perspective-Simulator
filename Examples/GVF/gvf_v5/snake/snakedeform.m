function [x,y] = snakedeform(x,y,alpha,beta,gamma,kappa,fx,fy,ITER)
% SNAKEDEFORM  Deform snake in the given external force field
%     [x,y] = snakedeform(x,y,alpha,beta,gamma,kappa,fx,fy,ITER)
%
%     alpha:   elasticity parameter
%     beta:    rigidity parameter
%     gamma:   viscosity parameter
%     kappa:   external force weight
%     fx,fy:   external force field

%      Chenyang Xu and Jerry L. Prince, 4/1/95, 6/17/97
%      Copyright (c) 1995-97 by Chenyang Xu and Jerry L. Prince
%      Image Analysis and Communications Lab, Johns Hopkins University


% generates the parameters for snake

N = length(x);

alpha = alpha* ones(1,N); 
beta = beta*ones(1,N);
    
% produce the five diagnal vectors
alpham1 = [alpha(2:N) alpha(1)];
alphap1 = [alpha(N) alpha(1:N-1)];
betam1 = [beta(2:N) beta(1)];
betap1 = [beta(N) beta(1:N-1)];

% t = linspace(0,1,N/2);
% t = [t fliplr(t)];
% alpham1 = alpha.*t;
% alphap1 = alpha.*t;
% betam1 = beta.*t;
% betap1 = beta.*t;


a = betam1;
b = -alpha - 2*beta - 2*betam1;
c = alpha + alphap1 +betam1 + 4*beta + betap1;
d = -alphap1 - 2*beta - 2*betap1;
e = betap1;

% generate the parameters matrix
A = diag(a(1:N-2),-2) + diag(a(N-1:N),N-2);
A = A + diag(b(1:N-1),-1) + diag(b(N), N-1);
A = A + diag(c);
A = A + diag(d(1:N-1),1) + diag(d(N),-(N-1));
A = A + diag(e(1:N-2),2) + diag(e(N-1:N),-(N-2));

% Fix the start & endpoints 
A(1,:) = 0; A(2,:) = 0; A(2,1:3) = [1 -2 1];
A(end,:) = 0; A(end-1,:) = 0; A(end-1,(end-2):(end-0)) = [1 -2 1];

% invAI = inv(A + gamma * diag(ones(1,N)));
% Use LU decomposition to speed up the matrix inverse calculation
[L,U] = lu(A + gamma * diag(ones(1,N)));
invAI = inv(U) * inv(L);

h = plot(x,y,'m');
for count = 1:ITER,
   vfx = interp2(fx,x,y,'*linear');
   vfy = interp2(fy,x,y,'*linear');

%    % For points outside of the image, bound the snake to 0 velocity
%    vfx = interp2(fx,x,y,'*linear', 0);
%    vfy = interp2(fy,x,y,'*linear', 0);

   % deform snake
   x = invAI * (gamma* x + kappa*vfx);
   y = invAI * (gamma* y + kappa*vfy);
   set(h, 'XData', x(:), 'YData', y(:));
   1;%
end
