function [J, J_real, x] = estimate_cost(K, W, x0, xo, x_set, Black_Box_Env, TJ)
[T,~] = size(xo);

if T == 1
%initialization
assert(length(K) == 3*length(W))
mu = zeros(size(x0));
x = x0;
l = length(W);
K_mat = blkdiag(K{1:l});
Ko_vec = [K{l+1:2*l}]';
b = [K{2*l+1:3*l}]';
J_real = 0;
for t = 1:TJ
    u = K_mat*x + Ko_vec*xo + b;
    [c, x] = Black_Box_Env(x, u, xo, x_set);
    J_real = J_real + sum(c);
    mu = (t-1)/t*W*mu + 1/t*c;
end
J = (length(K)-1)*mu;
J_real = J_real/TJ;

else
assert(T == TJ)
assert(length(K) == 3*length(W))
mu = zeros(size(x0));
x = x0;
l = length(W);
K_mat = blkdiag(K{1:l});
Ko_vec = [K{l+1:2*l}]';
b = [K{2*l+1:3*l}]';
J_real = 0;
for t = 1:TJ
    u = K_mat*x + Ko_vec*xo(t) + b;
    [c, x] = Black_Box_Env(x, u, xo(t), x_set);
    J_real = J_real + sum(c);
    mu = (t-1)/t*W*mu + 1/t*c;
end
J = (length(K)-1)*mu;
J_real = J_real/TJ;
end

end
