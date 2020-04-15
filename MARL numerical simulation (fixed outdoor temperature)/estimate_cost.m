function [J, J_real] = estimate_cost(K, W, x0, Black_Box_Env, TJ)

%initialization
assert(length(K) == length(W)+1)
mu = zeros(size(x0));
x = x0;
K_mat = blkdiag(K{1:end-1});
b = K{end};
J_real = 0;
for t = 1:TJ
    u = K_mat*x + b;
    [c, x] = Black_Box_Env(x, u);
    J_real = J_real + sum(c);
    mu = (t-1)/t*W*mu + 1/t*c;
end
J = (length(K)-1)*mu;
J_real = J_real/TJ;
end
