%%
% File name : prediction.m
% Author : Zhengqing GONG
% Date : 28/04/2022
% Version : 1.0

%%
function u_k= prediction(x_k, E, H, N, p)

U_k = zeros(N*p, 1); % NP x 1

U_k = quadprog(H, E*x_k);

u_k = U_k(1:p,1);

end