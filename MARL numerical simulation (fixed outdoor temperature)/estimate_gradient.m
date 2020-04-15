function g = estimate_gradient(cost_estimator, sample_generator, K, args)
% Use zero-order optimization to estimate the gradient \nabla J(K)
% cost_estimator
% sample_generator: generate samples that are uniformly distributed on a
% unit sphere
% K: the controller (A cell that can be converted to blkdiag matrix wih command blkdiag(K{:}))
% args: TB - number of batches
%       r - radius in the zero-order optimization

% initialization
TB = args.TB;
r = args.r;
g = zero_cell(K);
nm = numel_cell(K);
% gradient estimation
for step = 1:TB
    D_step = sample_generator();
    K_step = cell_add(K, scalar_cell_mult(r, D_step));
    J_step = cost_estimator(K_step);
    %J = cost_estimator(K);
    %J_step = min(J_step, 37);
    g = cell_add(g, vector_cell_mult2(J_step, scalar_cell_mult(nm/r, D_step)));
end
g = scalar_cell_mult(1/TB, g);
end