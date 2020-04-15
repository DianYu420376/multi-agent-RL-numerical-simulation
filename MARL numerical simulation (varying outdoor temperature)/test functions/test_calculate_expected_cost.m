function [cost,cost_lst] = test_calculate_expected_cost(K, xo_lst,x_set,in_door_gain, A, B, sigma, alpha, delta_t, C)
L = length(xo_lst);
cost_lst = zeros(L,1);
N = length(K)/3;
for i = 1:L
xo = xo_lst(i);
A_tilde = eye(N) + 1/C*delta_t*A; B_tilde = 1/C*delta_t*B;
Sigma_w = eye(N) * 1/C*1/C*delta_t * sigma^2; Q = eye(N); R = alpha*eye(N);
d = 1/C*delta_t*(xo+in_door_gain);theta = x_set*ones(N,1);
new_K = K(1:N);
b1 = [K{N+1:2*N}]';
b2 = [K{2*N+1:3*N}]';
b = b1*xo + b2;
new_K{N+1} = b;
cost = test_calculate_cost(A_tilde, B_tilde,d,theta, Sigma_w, Q, R, new_K);
cost_lst(i) = cost;
end
cost = mean(cost_lst);