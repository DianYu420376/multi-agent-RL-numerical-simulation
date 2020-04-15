function g = test_calculate_gradient(A, B,d,theta, Sigma_w, Q, R, K)
K_mat = blkdiag(K{1:end-1}); b = K{end};
N = size(K_mat,1);
%x_tilde = (eye(N) - A)\d;
%theta = theta - x_tilde;
%b = K_mat*x_tilde + b;

A_tilde = A + B*K_mat;
Sigma_K = dlyap(A_tilde, Sigma_w);
%Sigma_K1 = dlyap(A + B*K_mat, Sigma_w);
%mu = (eye(N) - (A+B*K_mat))\(B*b);
mu = (eye(N) - (A+B*K_mat))\(B*b + d);
Kmub = K_mat*mu + b;
M = inv(eye(N) - A_tilde)*B;

temp_mat = (Q + K_mat'*R*K_mat);
PK = dlyap(A_tilde, temp_mat);
nabla_JK_K = 2*(R*K_mat*Sigma_K + B'*PK*A_tilde*Sigma_K) + ...
    (2*mu*((mu-theta)'*Q*M) + 2*mu*(Kmub'*R) + 2*mu*(Kmub'*R*K_mat*M))';

nabla_JK_b = 2*(M'*(Q*(mu - theta))) + 2*(K_mat*M + eye(N))'*(R*Kmub);
g = cell(size(K));
%col_start = 0;
col_end = 0;
for i = 1:length(K)-1
    col_start = col_end + 1;
    n_i = size(K{i}, 1);
    assert(size(K{i},2) == n_i)
    col_end = col_start - 1 + n_i;
    g{i} = nabla_JK_K(col_start:col_end, col_start:col_end);
end
g{end} = nabla_JK_b;
end