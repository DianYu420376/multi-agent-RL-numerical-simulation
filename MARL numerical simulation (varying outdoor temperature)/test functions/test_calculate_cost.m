function cost = test_calculate_cost(A, B,d,theta, Sigma_w, Q, R, K)
K_mat = blkdiag(K{1:end-1}); b = K{end};
N = size(K_mat,1);
x_tilde = (eye(N) - A)\d;
theta = theta - x_tilde;
b = K_mat*x_tilde + b;

mu = (eye(N) - (A+B*K_mat))\(B*b);
%mu = (eye(N) - (A+B*K_mat))\(B*b + d);
Kmub = K_mat*mu + b;

Sigma_K = dlyap(A + B*K_mat, Sigma_w);
temp_mat = (Q + K_mat'*R*K_mat);

cost = trace(temp_mat*Sigma_K) + (mu-theta)'*Q*(mu-theta) + Kmub'*R*Kmub;
if trace(temp_mat*Sigma_K) <= 0
    pass
end
end