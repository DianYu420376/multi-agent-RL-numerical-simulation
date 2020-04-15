function g = estimate_gradient_coordinate(cost_estimator,sample_generator ,K, args)
% This is just a test function to help testing whether the function
% estimate_gradient is correct. Here I just assume K{i} is only a 1*1
% matrix(scalar)
r = args.r;
TB = args.TB;
N = length(K);
J = cost_estimator(K);
g = zero_cell(K);
for j = 1:TB
for i = 1:N
    K_step = K;
    K_step{i} = K{i} + r;
    J_step = cost_estimator(K_step);
    g{i} = g{i} + (J_step(i) - J(i))/r;
end
end
g = scalar_cell_mult(1/TB, g);
end
