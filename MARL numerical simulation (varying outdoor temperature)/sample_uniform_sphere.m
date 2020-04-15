function D = sample_uniform_sphere(K, W, TJ)

N = length(K);
assert(N == 3*length(W));
D = cell(1,N);
q = zeros(N,1);
% sample from normal distribution
for i = 1:N
    Ki = K{i};
    D{i} = randn(size(Ki));
    q(i) = norm(D{i}, 'fro')^2;
end

% first use centralized version to see if it works
%Norm = sqrt(sum(q));
%D = scalar_cell_mult(1/Norm, D);
%% communicate the norm
for t = 1:TJ
    q = blkdiag(W,W,W)*q;
end
q = sqrt(N*q);
D = vector_cell_mult(1./q, D);
end