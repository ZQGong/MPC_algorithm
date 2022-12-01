%%
% File name : main.m
% Author : Zhengqing GONG
% Date : 28/04/2022
% Version : 1.0
% This project is a practical project of the basic MPC algorithm of course
% ARS1： Advanced methods for dynamic system control

%% RAZ
clear;
close all;
clc;

%% If you use Octave please uncomment this line to load the optim package
% pkg load optim;

%% State matrix A (n,n) & B (n,p)
A = [[1  0.1];...
     [-1   2]];
n = size(A, 1);

B = [[0.2  1];...
     [0.5  2]];
p = size(B, 2);

%% Define matrix Q (n,n), F (n,n) & R (p,p)
Q = [[100 0];...
     [0   1]];

F = [[100 0];...
     [0   1]];

R = [[1  0];...
     [0 0.1]];

%% Iteration times
iter = 100;

%% State set 
x_k = zeros(n, iter);

%% Initial state (n,1)
x_k(:, 1) = [20; -20];

%% Input matrix (p,k)
u_k = zeros(p, iter);


%% Define the prediction horizon
N = 5;

%% Call MPC_Matrices fn to get matrices E & H
[E, H] = MPC_matrices(A, B, Q, R, F, N);



%% Update the state
for k = 1 : iter
    % Call prediction fn to get the optimal input u_k
    u_k(:,k) = prediction(x_k(:,k), E, H, N, p);

    % Calculate the state of step k+1
    x_k(:,k+1) = (A*x_k(:,k) + B*u_k(:,k));

end



%% Plot the state evolution

subplot(2, 1, 1);
hold;

for i = 1 : size(x_k,1)
    plot (x_k(i,:));
end
legend("x1", "x2")

hold off;

subplot (2, 1, 2);
hold;

for i = 1 : size(u_k,1)
    plot (u_k(i,:));
end
legend("u1", "u2")







