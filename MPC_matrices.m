%%
% File name : MPC_matrices.m
% Author : Zhengqing GONG
% Date : 28/04/2022
% Version : 1.0

%%
function[E, H] = MPC_matrices(A, B, Q, R, F, N)

%% Initialization
% Size of state matrices
n = size(A, 1);
p = size(B, 2);

% Initialize matrix M,   Dimension: ((N+1)*n, n)
M = [eye(n);
     zeros(N*n,n)];

% Initialize matrix C,  Dimension: ((N+1)*n, N*p),  with all zero
C = zeros((N+1)*n, N*p); 

I = eye(n);

%% Iteration within the prediction horizon to generate the M & C
for i = 1:N
    rows = i*n + (1:n); 
    C(rows, :) = [I*B, C(rows-n, 1:end-p)];
    I = A*I;
    M(rows, :)=I;
end


%% Define Q_bar & R_bar

Q_bar = kron(eye(N), Q);

Q_bar = blkdiag(Q_bar, F);

R_bar = kron(eye(N), R);


%% Calculate G, E & H

G = M'*Q_bar*M; % (n, n)

E = C'*Q_bar*M; % (NP, n)

H = C'*Q_bar*C+R_bar; % (NP, NP) 


end